import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray

import groundingdino
import groundingdino.models
import groundingdino.datasets.transforms
import segment_anything
import transformers
import numpy as np
import torchvision
import PIL.Image
import torch
import os

import ros2_numpy


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision")

        self.declare_parameter("caption", "soup can")
        self.param_caption = (
            self.get_parameter("caption").get_parameter_value().string_value
        )

        self.declare_parameter("image", rclpy.Parameter.Type.STRING)
        self.param_image = (
            self.get_parameter("image").get_parameter_value().string_value
        )

        self.declare_parameter("image_bbox", rclpy.Parameter.Type.STRING)
        self.param_image_bbox = (
            self.get_parameter("image_bbox").get_parameter_value().string_value
        )

        self.declare_parameter("image_masks", rclpy.Parameter.Type.STRING)
        self.param_image_masks = (
            self.get_parameter("image_masks").get_parameter_value().string_value
        )

        self.declare_parameter("image_depth", rclpy.Parameter.Type.STRING)
        self.param_image_depth = (
            self.get_parameter("image_depth").get_parameter_value().string_value
        )

        self.declare_parameter("bbox", rclpy.Parameter.Type.STRING)
        self.param_bbox = self.get_parameter("bbox").get_parameter_value().string_value

        self.caption = self.param_caption
        self.box_threshold = 0.3
        self.text_threshold = 0.25

        self.caption = self.caption.lower().strip()
        self.caption = (
            self.caption + "." if not self.caption.endswith(".") else self.caption
        )

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        config = os.path.abspath(
            os.path.join(
                os.path.dirname(groundingdino.__file__),
                "config/GroundingDINO_SwinT_OGC.py",
            )
        )
        checkpoint_sam = "/opt/ros/local/model/segment-anything/sam_vit_h_4b8939.pth"
        checkpoint_dino = (
            "/opt/ros/local/model/GroundingDINO/groundingdino_swint_ogc.pth"
        )
        args = groundingdino.util.slconfig.SLConfig.fromfile(config)
        args.text_encoder_type = "/opt/ros/local/model/bert-base-uncased"
        args.device = self.device
        sam_version = "vit_h"

        self.model_dino = groundingdino.models.build_model(args)
        checkpoint = torch.load(checkpoint_dino, map_location="cpu")
        print(
            self.model_dino.load_state_dict(
                groundingdino.util.utils.clean_state_dict(checkpoint["model"]),
                strict=False,
            )
        )
        self.model_dino.eval()
        self.model_dino.to(self.device)

        self.model_sam = segment_anything.SamPredictor(
            segment_anything.sam_model_registry[sam_version](
                checkpoint=checkpoint_sam
            ).to(self.device)
        )

        self.model_dpt = transformers.DPTForDepthEstimation.from_pretrained(
            "/opt/ros/local/model/dpt-large"
        )
        self.processor_dpt = transformers.DPTImageProcessor.from_pretrained(
            "/opt/ros/local/model/dpt-large"
        )

        self.get_logger().info(
            f"I model: {self.model_dino} {self.model_sam} {self.model_dpt}"
        )

        self.subscription = self.create_subscription(
            Image,
            self.param_image,
            self.subscription_callback,
            10,
        )
        self.publisher_bbox = self.create_publisher(
            Detection2DArray, f"{self.param_bbox}", 10
        )
        self.publisher_image_bbox = self.create_publisher(
            Image, f"{self.param_image_bbox}", 10
        )
        self.publisher_image_masks = self.create_publisher(
            Image, f"{self.param_image_masks}", 10
        )
        self.publisher_image_depth = self.create_publisher(
            Image, f"{self.param_image_depth}", 10
        )

    def subscription_callback(self, msg):
        data = ros2_numpy.numpify(msg)
        match msg.encoding:
            case "bgr8":
                data = data[..., ::-1]
            case "rgb8":
                pass
            case other:
                raise TypeError(f"Image {other} not supported")
        image = PIL.Image.fromarray(data)
        self.get_logger().info(f"I heard: {data.dtype} {data.shape} => {image}")

        cxcywh, xyxy, boxes, logits, masks, depth = self.callback(image)
        self.get_logger().info(f"I cxcywh: {cxcywh} {xyxy} {boxes}")

        def f(e):
            detection = Detection2D()
            (
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.size_x,
                detection.bbox.size_y,
            ) = e
            return detection

        detections = Detection2DArray()
        detections.header.frame_id = msg.header.frame_id
        detections.detections = list(
            map(
                f,
                cxcywh.tolist(),
            )
        )
        self.publisher_bbox.publish(detections)
        self.get_logger().info(f'Publishing: "{detections}"')

        image = torch.from_numpy(data.copy()).permute(2, 0, 1)

        bbox = torchvision.utils.draw_bounding_boxes(image, xyxy, fill=True).permute(
            1, 2, 0
        )
        bbox = ros2_numpy.msgify(Image, bbox.numpy(), encoding="rgb8")
        self.publisher_image_bbox.publish(bbox)

        masked = torchvision.utils.draw_segmentation_masks(
            image, torch.squeeze(masks, dim=1)
        ).permute(1, 2, 0)
        masked = ros2_numpy.msgify(Image, masked.numpy(), encoding="rgb8")
        self.publisher_image_masks.publish(masked)

        formatted = depth.squeeze(1).permute(1, 2, 0).numpy()
        formatted = (formatted * 255 / np.max(formatted)).astype("uint8")
        formatted = ros2_numpy.msgify(Image, formatted, encoding="mono8")
        self.publisher_image_depth.publish(formatted)

    def callback(self, input):
        def transform_dino(i):
            image_transformed, _ = groundingdino.datasets.transforms.Compose(
                [
                    groundingdino.datasets.transforms.RandomResize(
                        [800], max_size=1333
                    ),
                    groundingdino.datasets.transforms.ToTensor(),
                    groundingdino.datasets.transforms.Normalize(
                        [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                    ),
                ]
            )(i, None)
            return image_transformed

        image = np.array(input)
        image_transformed = transform_dino(input).to(self.device)
        self.get_logger().info(
            f"transform: {image.dtype} {image.shape} => {image_transformed.dtype} {image_transformed.shape}"
        )

        with torch.no_grad():
            outputs = self.model_dino(image_transformed[None], captions=[self.caption])
            logits, boxes = (
                outputs["pred_logits"].sigmoid().max(dim=2).values,
                outputs["pred_boxes"],
            )
            filtered = logits > self.box_threshold
            logits_filtered = logits[filtered]
            boxes_filtered = boxes[filtered]
            boxes_cxcywh = boxes_filtered * torch.Tensor(
                [image.shape[1], image.shape[0], image.shape[1], image.shape[0]]
            ).to(device=self.device)
            boxes_xyxy = torchvision.ops.box_convert(boxes_cxcywh, "cxcywh", "xyxy")

            boxes_filtered = groundingdino.util.box_ops.box_cxcywh_to_xyxy(
                boxes_filtered
            ) * torch.Tensor(
                [image.shape[1], image.shape[0], image.shape[1], image.shape[0]]
            ).to(
                device=self.device
            )

            self.model_sam.set_image(image)
            boxes_transformed = self.model_sam.transform.apply_boxes_torch(
                boxes_filtered, image.shape[:2]
            ).to(self.device)
            masks, _, _ = self.model_sam.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=boxes_transformed,
                multimask_output=False,
            )

            depth = self.model_dpt(
                **self.processor_dpt(images=input, return_tensors="pt")
            ).predicted_depth
            depth = torch.nn.functional.interpolate(
                depth.unsqueeze(1),
                size=input.size[::-1],
                mode="bicubic",
                align_corners=False,
            )

        self.get_logger().info(
            f"boxes_cxcywh: {boxes_cxcywh.dtype} ({boxes_cxcywh.shape}) boxes: {boxes_filtered.dtype} ({boxes_filtered.shape}) logits: {logits_filtered.dtype} ({logits_filtered.shape}) masks: {masks.dtype} ({masks.shape}) depth: {depth.dtype} ({depth.shape})"
        )
        return boxes_cxcywh, boxes_xyxy, boxes_filtered, logits_filtered, masks, depth


def main(args=None):
    rclpy.init(args=args)

    node = VisionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
