import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import subprocess
from datetime import datetime
from gpiozero import DistanceSensor
from time import time, sleep

# --- Ultrasonic sensor setup ---

# --- Helper functions ---

def create_run_folder(results_root):
    """Create timestamped run folder with raw/processed subfolders."""
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = os.path.join(results_root, run_id)

    raw_dir = os.path.join(run_folder, "raw")
    processed_dir = os.path.join(run_folder, "processed")

    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(processed_dir, exist_ok=True)

    return run_folder, raw_dir, processed_dir

# --- Camera capture ---
def take_picture(raw_dir):
    """Take a picture and save in the raw folder."""
    filename = os.path.join(raw_dir, "raw_image.png")
    cmd = f"rpicam-still -o {filename}"
    subprocess.run(cmd, shell=True, check=True)
    print(f"Picture taken: {filename}")
    return filename

# --- Image processing functions ---
def grayscale(path):
    image = cv2.imread(path)
    if image is None:
        raise FileNotFoundError(f"Image not found at {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def morphological_cleanup(image, kernel_size=3, open_iter=1, close_iter=1):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    cleaned = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel, iterations=open_iter)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    return cleaned

def adaptive_threshold(gray_img):
    return cv2.adaptiveThreshold(
        gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 2
    )

def create_markers(binary_img):
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
    markers, _, _, _ = create_markers(binary_img)
    if len(original_img.shape) == 2:
        original_color = cv2.cvtColor(original_img, cv2.COLOR_GRAY2BGR)
    else:
        original_color = original_img.copy()
    cv2.watershed(original_color, markers)
    segmented = original_color.copy()
    segmented[markers == -1] = [0, 255, 0]
    return segmented, markers

def compute_grain_size_pixels(markers, pixel_to_mm=None):
    unique, counts = np.unique(markers[markers > 1], return_counts=True)
    areas = counts
    if pixel_to_mm:
        areas = areas * (pixel_to_mm ** 2)
    return areas

def compute_mm_per_pixel(distance_mm, sensor_mm, focal_length_mm, image_pixels):
    return (distance_mm * sensor_mm) / (focal_length_mm * image_pixels)

# --- Full pipeline ---
def run_pipeline(system_dir='/home/rapi6/Biomimicry_group6/System', image_path=None):
    results_dir = os.path.join(system_dir, 'results_image_capture')
    os.makedirs(results_dir, exist_ok=True)

    # --- Create run folders ---
    run_folder, raw_dir, processed_dir = create_run_folder(results_dir)

    # --- Take picture if needed ---
    if image_path is None:
        image_path = take_picture(raw_dir)

    # --- Load & preprocess ---
    gray = grayscale(image_path)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    clahe_img = clahe.apply(gray)
    th = adaptive_threshold(clahe_img)
    cleaned = morphological_cleanup(th)
    original = cv2.imread(image_path)
    segmented, markers = apply_watershed(original, cleaned)

    # --- Real-world scaling ---
    distance_mm = 64.16 
    px_to_mm = compute_mm_per_pixel(distance_mm, 7.9, 6, original.shape[1])

    areas_mm2 = compute_grain_size_pixels(markers, pixel_to_mm=px_to_mm)
    diameters_mm = 2 * np.sqrt(areas_mm2 / np.pi)

    # --- Compute percentiles and statistics ---
    percentiles = np.arange(10, 101, 10)

    # Areas
    area_percentiles = np.percentile(areas_mm2, percentiles)
    area_stats = dict(zip([f"D{p}" for p in percentiles], area_percentiles))
    M50_area = np.median(areas_mm2)
    mean_area = np.mean(areas_mm2)
    std_area = np.std(areas_mm2)
    cv_area = std_area / mean_area if mean_area > 0 else 0

    # Diameters
    diameter_percentiles = np.percentile(diameters_mm, percentiles)
    diameter_stats = dict(zip([f"D{p}" for p in percentiles], diameter_percentiles))
    M50_diam = np.median(diameters_mm)

    # --- Save images ---
    cv2.imwrite(os.path.join(processed_dir, "segmented.png"), segmented)

    # --- Save a figure of all processing steps ---
    steps = [original, gray, clahe_img, th, cleaned, segmented]
    labels = ["Original", "Grayscale", "CLAHE", "Thresholded", "Morph. Cleaned", "Segmented"]

    plt.figure(figsize=(18,3))
    for i, (img, label) in enumerate(zip(steps, labels), start=1):
        plt.subplot(1,6,i)
        if len(img.shape) == 2:  # grayscale
            plt.imshow(img, cmap='gray')
        else:
            plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.title(label)
        plt.axis('off')
    plt.tight_layout()
    plt.savefig(os.path.join(processed_dir, "processing_steps.png"))
    plt.close()

    # Histogram (area)
    bins = np.logspace(np.log10(areas_mm2.min()), np.log10(areas_mm2.max()), 20)
    hist, bin_edges = np.histogram(areas_mm2, bins=bins)
    plt.figure()
    plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), align='edge')
    plt.xscale('log')
    plt.xlabel('Grain area (mm²)')
    plt.ylabel('Count')
    plt.title('Grain Size Distribution')
    plt.savefig(os.path.join(processed_dir, "grain_histogram.png"))
    plt.close()

    # --- Save stats & CSV ---
    with open(os.path.join(run_folder, "stats.txt"), "w") as f:
        f.write("=== Grain area statistics (mm²) ===\n")
        for label, val in area_stats.items():
            f.write(f"{label}: {val:.2f}\n")
        f.write(f"M50 (median): {M50_area:.2f}\n")
        f.write(f"Mean area: {mean_area:.2f}\n")
        f.write(f"Std area: {std_area:.2f}\n")
        f.write(f"CV area: {cv_area:.3f}\n\n")

        f.write("=== Equivalent diameter statistics (mm) ===\n")
        for label, val in diameter_stats.items():
            f.write(f"{label}: {val:.2f}\n")
        f.write(f"M50 (median): {M50_diam:.2f}\n\n")

        f.write("=== Measurement info ===\n")
        f.write(f"Number of grains: {len(areas_mm2)}\n")
        f.write(f"Measured distance: {distance_mm:.2f} mm\n")
        f.write(f"mm per pixel: {px_to_mm:.6f} mm/px\n")

    np.savetxt(os.path.join(run_folder, "grain_areas_mm2.csv"), areas_mm2,
               delimiter=',', header='grain_area_mm2', comments='')

    np.savetxt(os.path.join(run_folder, "grain_diameters_mm.csv"), diameters_mm,
               delimiter=',', header='grain_diameter_mm', comments='')

    print(f"\n--- RESULTS SAVED ---")
    print(f"Run folder: {run_folder}")
    print(f"Number of grains: {len(areas_mm2)}")
    print(f"Measured distance: {distance_mm:.2f} mm")
    print(f"mm per pixel: {px_to_mm:.6f} mm/px\n")

    print("=== Grain area percentiles (mm²) ===")
    for label, val in area_stats.items():
        print(f"{label}: {val:.2f}")
    print(f"M50 (median area): {M50_area:.2f}")
    print(f"Mean area: {mean_area:.2f}")
    print(f"Std area: {std_area:.2f}")
    print(f"CV area: {cv_area:.3f}\n")

    print("=== Grain diameter percentiles (mm) ===")
    for label, val in diameter_stats.items():
        print(f"{label}: {val:.2f}")
    print(f"M50 (median diameter): {M50_diam:.2f}")

if __name__ == "__main__":
    run_pipeline()
