"""CPU-bounded FSOCO YOLO26 left-eye detector with geometric stereo matching.

Only one ONNX forward pass is needed per stereo pair. The right-eye match is
obtained from measured image content: color candidates first, then a local
epipolar template search. No CARLA cone actors or track data are consulted.
"""
import ast
import math
import os
from pathlib import Path

import cv2
import numpy as np

from .detection import Candidate, associate_candidates, candidates, nominal_cone_height

_MODEL = Path(__file__).resolve().parents[3] / "Models/yolo26n_fsoco_e2e_opset17.onnx"
_CLASSES = {"yellow": "yellow", "blue": "blue", "orange": "orange",
            "large_orange": "large_orange"}


def box_iou(a, b):
    overlap_w = max(0, min(a.x + a.w, b.x + b.w) - max(a.x, b.x))
    overlap_h = max(0, min(a.y + a.h, b.y + b.h) - max(a.y, b.y))
    intersection = overlap_w * overlap_h
    return intersection / max(1, a.w * a.h + b.w * b.h - intersection)


def decode_detections(rows, class_names, original_shape, confidence, limit=32,
                      model_width=640, model_height=640):
    """Decode the verified YOLO26 end-to-end Nx6 output to original pixels."""
    height, width = original_shape[:2]
    scale = min(model_width / width, model_height / height)
    resized_w = round(width * scale)
    resized_h = round(height * scale)
    left_pad = (model_width - resized_w) // 2
    top_pad = (model_height - resized_h) // 2
    found = []
    if rows.ndim != 2 or rows.shape[1] != 6:
        raise ValueError(f"Expected YOLO26 end-to-end Nx6 output; got {rows.shape}")
    for box in rows:
        x1, y1, x2, y2, score, class_id = [float(v) for v in box]
        if not all(math.isfinite(v) for v in (x1, y1, x2, y2, score, class_id)):
            continue
        if score < confidence or class_id != int(class_id):
            continue
        cls = _CLASSES.get(class_names.get(int(class_id), "unknown"))
        if cls is None:
            continue  # FSOCO unknown class has no compatible legacy ROS label
        x1 = max(0, min(width, (x1 - left_pad) / scale))
        x2 = max(0, min(width, (x2 - left_pad) / scale))
        y1 = max(0, min(height, (y1 - top_pad) / scale))
        y2 = max(0, min(height, (y2 - top_pad) / scale))
        x, y = int(x1), int(y1)
        w, h = int(math.ceil(x2)) - x, int(math.ceil(y2)) - y
        if w < 3 or h < 4:
            continue
        candidate = Candidate(cls, x, y, w, h, float(w * h), 100.0 * score)
        if any(c.color == candidate.color and box_iou(c, candidate) > .60 for c in found):
            continue
        found.append(candidate)
        if len(found) >= limit:
            break
    return found


def right_epipolar_match(left, left_gray, right_gray, baseline_m, threshold=.72):
    """Match one left-eye cone appearance within physically plausible disparities."""
    height, width = left_gray.shape
    if left.w < 5 or left.h < 7 or left.h > height // 2:
        return None
    x, y, w, h = left.x, left.y, left.w, left.h
    if x < 0 or y < 0 or x + w > width or y + h > height:
        return None
    template = left_gray[y:y + h, x:x + w]
    if float(template.std()) < 7:
        return None
    expected = baseline_m * h / nominal_cone_height(left.color)
    min_disp = max(.75, expected * .38)
    max_disp = max(2.8, expected * 2.25)
    y_slack = max(3.5, h * .27)
    # Search the right image around the same row, towards smaller image u.
    sx = max(0, int(math.floor(x - max_disp)))
    ex = min(width, int(math.ceil(x - min_disp + w)))
    sy = max(0, int(math.floor(y - y_slack)))
    ey = min(height, int(math.ceil(y + y_slack + h)))
    region = right_gray[sy:ey, sx:ex]
    if region.shape[0] < h or region.shape[1] < w:
        return None
    response = cv2.matchTemplate(region, template, cv2.TM_CCOEFF_NORMED)
    _, peak, _, (dx, dy) = cv2.minMaxLoc(response)
    if not math.isfinite(peak) or peak < threshold:
        return None
    rx, ry = sx + dx, sy + dy
    disparity = left.u - (rx + w / 2)
    if not min_disp <= disparity <= max_disp:
        return None
    return Candidate(left.color, rx, ry, w, h, float(w * h))


class Yolo26Stereo:
    def __init__(self, model_path=None, confidence=.30, cpu_threads=2,
                 max_detections=32, template_threshold=.72):
        import onnxruntime as ort  # available only to camera via its local dependency path
        if not .05 <= confidence <= .95:
            raise ValueError("YOLO confidence must be between 0.05 and 0.95")
        if not 1 <= cpu_threads <= 8:
            raise ValueError("YOLO CPU threads must be between 1 and 8")
        self.confidence = float(confidence)
        self.max_detections = int(max_detections)
        self.template_threshold = float(template_threshold)
        options = ort.SessionOptions()
        options.intra_op_num_threads = int(cpu_threads)
        options.inter_op_num_threads = 1
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        provider = ("CUDAExecutionProvider" if os.environ.get("ASURT_INFERENCE_BACKEND") == "cuda"
                    else "CPUExecutionProvider")
        providers = ([(provider, {"cudnn_conv_algo_search": "HEURISTIC"})]
                     if provider == "CUDAExecutionProvider" else [provider])
        self.session = ort.InferenceSession(str(model_path or _MODEL), sess_options=options,
                                            providers=providers)
        if provider not in self.session.get_providers():
            raise RuntimeError(f"YOLO26 could not initialize {provider}")
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape != [1, 3, 640, 640]:
            raise ValueError("YOLO26 requires one fixed 1x3x640x640 input")
        if len(outputs) != 1 or outputs[0].shape != [1, 300, 6]:
            raise ValueError("Expected YOLO26 end-to-end 1x300x6 output")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        metadata = self.session.get_modelmeta().custom_metadata_map
        names = ast.literal_eval(metadata["names"])
        self.class_names = {int(k): str(v) for k, v in names.items()}
        expected = {0: "yellow", 1: "blue", 2: "orange",
                    3: "large_orange", 4: "unknown"}
        if self.class_names != expected or metadata.get("end2end") != "True":
            raise ValueError(f"Unrecognized YOLO26 FSOCO export metadata: {metadata}")

    @staticmethod
    def preprocess(image):
        height, width = image.shape[:2]
        ratio = min(640 / width, 640 / height)
        rw, rh = round(width * ratio), round(height * ratio)
        pad_x = (640 - rw) // 2
        pad_y = (640 - rh) // 2
        rgb = cv2.cvtColor(cv2.resize(image, (rw, rh)), cv2.COLOR_BGR2RGB)
        padded = np.full((640, 640, 3), 114, dtype=np.uint8)
        padded[pad_y:pad_y + rh, pad_x:pad_x + rw] = rgb
        return np.ascontiguousarray(padded.transpose(2, 0, 1)[None],
                                    dtype=np.float32) / 255.0

    def left_candidates(self, image):
        blob = self.preprocess(image)
        prediction = self.session.run([self.output_name],
                                      {self.input_name: blob})[0][0]
        prediction = prediction[np.argsort(-prediction[:, 4])]
        return decode_detections(prediction, self.class_names, image.shape,
                                 self.confidence, self.max_detections)

    def detect_stereo(self, left_bgr, right_bgr, calibration):
        left = self.left_candidates(left_bgr)
        if not left:
            return []
        # Cheap right-eye color segmentation where the cone remains visible.
        observations = associate_candidates(left, candidates(right_bgr), calibration)
        matched_boxes = {tuple(cone.box) for cone in observations}
        # When color is washed out in the right eye, match its real pixel
        # appearance locally along the epipolar line; never invent a depth.
        remaining = [c for c in left if (c.x, c.y, c.w, c.h) not in matched_boxes]
        if remaining:
            left_gray = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
            for cone in remaining:
                matched = right_epipolar_match(cone, left_gray, right_gray,
                                               calibration.baseline_m,
                                               self.template_threshold)
                if matched is not None:
                    observations.extend(associate_candidates([cone], [matched], calibration))
        return observations
