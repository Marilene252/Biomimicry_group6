import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import subprocess
from datetime import datetime
from gpiozero import DistanceSensor
from time import time, sleep

# --- Ultrasonic sensor setup ---
Trigger = 23
Echo = 24
sensor = DistanceSensor(echo=Echo, trigger=Trigger, max_distance=4)

def get_distance_mm(duration=3.0, tolerance_mm=2.0, max_mm=300):
    """Measure stable distance in mm using ultrasonic sensor."""
    while True:
        readings = []
        start = time()
        while time() - start < duration:
            d = sensor.distance * 1000
            if d > 0: readings.append(d)
            sleep(0.1)

        readings = np.array(readings)
        median = np.median(readings)
        stable = readings[np.abs(readings - median) < tolerance_mm]

        if len(stable) < 5:
            print("Distance not stable, retrying...")
            continue

        distance_mm = np.mean(stable)
        if distance_mm > max_mm:
            print(f"Distance {distance_mm:.1f} mm > {max_mm} mm, retrying...")
            continue
        return distance_mm

# --- Camera capture ---
def take_picture(system_dir='/home/rapi6/Biomimicry_group6/System'):
    """Take a picture and save it in the date folder."""
    # Create date folder
    date_folder = datetime.now().strftime("%Y-%m-%d")
    folder = os.path.join(system_dir, 'Results', date_folder)
    os.makedirs(folder, exist_ok=True)
    filename = os.path.join(folder, 'raw_image.jpg')
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
    th = cv2.adaptiveThreshold(
        gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 2
    )
    return th

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
    markers, sure_fg, sure_bg, unknown = create_markers(binary_img)
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
    results_dir = os.path.join(system_dir, 'Results')
    os.makedirs(results_dir, exist_ok=True)

    # Only take a picture if none provided
    if image_path is None:
        image_path = take_picture(results_dir)

    # Load & preprocess
    gray = grayscale(image_path)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    clahe_img = clahe.apply(gray)
    th = adaptive_threshold(clahe_img)
    cleaned = morphological_cleanup(th)
    original = cv2.imread(image_path)
    segmented, markers = apply_watershed(original, cleaned)

    # Real-world scaling
    distance_mm = get_distance_mm()
    px_to_mm = compute_mm_per_pixel(distance_mm, 7.9, 6, original.shape[1])

    # Compute grain areas
    areas_mm2 = compute_grain_size_pixels(markers, pixel_to_mm=px_to_mm)
    M50 = np.median(areas_mm2)
    D10 = np.percentile(areas_mm2, 10)
    D50 = np.percentile(areas_mm2, 50)
    D90 = np.percentile(areas_mm2, 90)

    # Use a fixed folder for this “date” run
    date_folder = datetime.now().strftime("%Y-%m-%d")
    folder = os.path.join(results_dir, date_folder)
    os.makedirs(folder, exist_ok=True)

    # Save processed images
    cv2.imwrite(os.path.join(folder, 'raw_image.png'), original)
    cv2.imwrite(os.path.join(folder, 'segmented.png'), segmented)

    # Save stats
    with open(os.path.join(folder, 'stats.txt'), 'w') as f:
        f.write(f"M50: {M50:.2f} mm²\n")
        f.write(f"D10: {D10:.2f} mm²\n")
        f.write(f"D50: {D50:.2f} mm²\n")
        f.write(f"D90: {D90:.2f} mm²\n")
        f.write(f"Number of grains: {len(areas_mm2)}\n")
        f.write(f"mm per pixel: {px_to_mm:.6f} mm/px\n")

    # Histogram
    bins = np.logspace(np.log10(areas_mm2.min()), np.log10(areas_mm2.max()), 20)
    hist, bin_edges = np.histogram(areas_mm2, bins=bins)
    plt.figure()
    plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor='black', align='edge')
    plt.xscale('log')
    plt.xlabel('Grain area (mm²)')
    plt.ylabel('Count')
    plt.title('Grain Size Histogram')
    plt.savefig(os.path.join(folder, 'grain_histogram.png'))
    plt.close()

    # Save histogram CSV
    np.savetxt(
        os.path.join(folder, 'bins.csv'),
        np.column_stack((bin_edges[:-1], hist)),
        delimiter=',',
        header='bin_start_mm2,count',
        comments=''
    )

    print(f"\n--- RESULTS SAVED ---\nFolder: {folder}")
    print(f"Detected grains: {len(areas_mm2)}")
    print(f"M50: {M50:.2f} mm², D10: {D10:.2f}, D50: {D50:.2f}, D90: {D90:.2f}")
    print(f"mm per pixel: {px_to_mm:.6f} mm/px")

if __name__ == "__main__":
    run_pipeline()
