import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('/home/marilene/github/Biomimicry_group6')
from distance_sensor import get_distance_mm

def grayscale(path):
    """Load image from path and convert to grayscale."""
    image = cv2.imread(path)
    if image is None:
        raise FileNotFoundError(f"Image not found at {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def morphological_cleanup(image, kernel_size=3, open_iter=1, close_iter=1):
    """Morphological open+close to remove small noise and fill holes."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    cleaned = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel, iterations=open_iter)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    return cleaned


def adaptive_threshold(gray_img):
    """Apply Adaptive Gaussian Thresholding (best for textured sand grains)."""
    th = cv2.adaptiveThreshold(
        gray_img,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,  # invert: grains white, background black
        15,  # block size — can be tuned (must be odd)
        2,   # constant subtracted from mean — adjust for lighting
    )
    return th


def create_markers(binary_img):
    """Create markers for watershed segmentation."""
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel, iterations=2)

    # Sure background
    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    # Sure foreground via distance transform
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist_transform, 0.4 * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)

    # Unknown region
    unknown = cv2.subtract(sure_bg, sure_fg)

    # Marker labeling
    _, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown == 255] = 0

    return markers, sure_fg, sure_bg, unknown


def apply_watershed(original_img, binary_img):
    """Apply watershed segmentation."""
    markers, sure_fg, sure_bg, unknown = create_markers(binary_img)

    if len(original_img.shape) == 2:
        original_color = cv2.cvtColor(original_img, cv2.COLOR_GRAY2BGR)
    else:
        original_color = original_img.copy()

    cv2.watershed(original_color, markers)

    segmented = original_color.copy()
    segmented[markers == -1] = [0, 255, 0]  

    return segmented, markers, sure_fg, sure_bg, unknown


def compute_grain_size_pixels(markers, pixel_to_mm=None):
    """Compute grain areas (in pixels)."""
    unique, counts = np.unique(markers[markers > 1], return_counts=True)
    areas = counts
    if pixel_to_mm:
        areas = areas * (pixel_to_mm ** 2)
    return areas

def compute_mm_per_pixel(distance_mm, sensor_mm, focal_length_mm, image_pixels):
    """
    Compute grain areas (in pixels)
    """
    return (distance_mm * sensor_mm) / (focal_length_mm * image_pixels)


def show_full_segmentation(path):
    """Run full sand segmentation pipeline using adaptive thresholding."""
    # 1. Load and preprocess
    gray = grayscale(path)

    # Contrast enhancement (helps with uneven lighting)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    clahe_img = clahe.apply(gray)

    # 2. Threshold + morphology
    th = adaptive_threshold(clahe_img)
    cleaned = morphological_cleanup(th)

    # 3. Watershed
    original = cv2.imread(path)
    segmented, markers, sure_fg, sure_bg, unknown = apply_watershed(original, cleaned)

    # 4. Display
    plt.figure(figsize=(12, 10))
    titles = [
        "Grayscale",
        "CLAHE Image",
        "Adaptive Gaussian Threshold",
        "After Morph Cleanup",
        "Sure Foreground",
        "Sure Background",
        "Unknown Region",
        "Watershed Segmentation",
    ]
    images = [gray, clahe_img, th, cleaned, sure_fg, sure_bg, unknown, segmented]

    for i, (img, title) in enumerate(zip(images, titles)):
        plt.subplot(3, 4, i + 1)
        if len(img.shape) == 2:
            plt.imshow(img, cmap="gray")
        else:
            plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.title(title)
        plt.axis("off")

    plt.tight_layout()
    plt.show()

    px_to_mm = compute_mm_per_pixel(get_distance_mm(), 7.9, 6, original.shape[1])

    print("\n--- REAL WORLD SCALE ---")
    print(f"mm per pixel: {px_to_mm:.6f} mm/px")

    # 5. Grain size stats
    areas = compute_grain_size_pixels(markers)
    print(f"Detected grains: {len(areas)}")
    print(f"Average grain area (pixels): {np.mean(areas):.2f}")

    return segmented, markers


if __name__ == "__main__":
    path = '/home/marilene/github/Biomimicry_group6/Vision/sand_pictures/sandsnap.jpg'
    show_full_segmentation(path)

