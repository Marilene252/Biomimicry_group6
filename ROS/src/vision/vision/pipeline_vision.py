import cv2
import numpy as np

def morphological_cleanup(image, kernel_size=3, open_iter=1, close_iter=1):
    """Morphological open+close to remove small noise and fill holes."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    cleaned = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel, iterations=open_iter)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    return cleaned


def adaptive_threshold(gray_img):
    """Apply Adaptive Gaussian Thresholding (best for textured sand grains)."""
    th = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 2,)
    return th

def create_markers(binary_img):
    """Create markers for watershed segmentation."""
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel, iterations=2)

    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist_transform, 0.4 * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)

    unknown = cv2.subtract(sure_bg, sure_fg)

    _, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown == 255] = 0

    return markers, sure_fg, sure_bg, unknown


def apply_watershed(original_img, binary_img):
    markers, sure_fg, sure_bg, unknown = create_markers(binary_img)

    if len(original_img.shape) == 2:
        original_color = cv2.cvtColor(original_img, cv2.COLOR_GRAY2BGR)
    else:
        original_color = original_img.copy()

    cv2.watershed(original_color, markers)

    segmented = original_color.copy()
    segmented[markers == -1] = [0, 255, 0]  

    return segmented, markers, sure_fg, sure_bg, unknown


def compute_grain_size_pixels(markers, pixel_to_mm):
    _, counts = np.unique(markers[markers > 1], return_counts=True)
    return counts * (pixel_to_mm ** 2)


def compute_mm_per_pixel(distance_mm, sensor_mm, focal_length_mm, image_pixels):
    return (distance_mm * sensor_mm) / (focal_length_mm * image_pixels)


def run_segmentation(image_bgr, distance_mm):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(2.0, (4, 4))
    enhanced = clahe.apply(gray)

    binary = adaptive_threshold(enhanced)
    cleaned = morphological_cleanup(binary)

    segmented, markers, *_ = apply_watershed(image_bgr, cleaned)

    px_to_mm = compute_mm_per_pixel(distance_mm, 7.9, 6.0, image_bgr.shape[1])
    areas_mm = compute_grain_size_pixels(markers, px_to_mm)

    m50 = float(np.median(areas_mm)) if len(areas_mm) else 0.0

    return segmented, m50, areas_mm, px_to_mm

