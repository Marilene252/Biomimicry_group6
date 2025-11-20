import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def show_full_segmentation(path, save_path=None):
    """Run full sand segmentation pipeline using adaptive thresholding."""
    # 1. Load and preprocess
    gray = grayscale(path)

    # Contrast enhancement
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

    # 5. Grain size stats
    areas = compute_grain_size(markers)
    print(f"Detected grains: {len(areas)}")
    print(f"Average grain area (pixels): {np.mean(areas):.2f}")

    # 6. Save watershed result if a save_path is provided
    if save_path:
        cv2.imwrite(save_path, segmented)
        print(f"Watershed result saved at: {save_path}")

    return segmented, markers


if __name__ == "__main__":
    path = '/home/marilene/github/Biomimicry_group6/Vision/sand_pictures/sandsnap.jpg'
    save_path = '/home/marilene/Desktop/watershed_result.png'  # where you want to save it
    show_full_segmentation(path, save_path=save_path)







for filename in os.listdir('/home/marilene/github/Biomimicry_group6/Vision'):
    if filename.endswith(".jpg"):  
        image_path = os.path.join('/home/marilene/github/Biomimicry_group6/Vision', 'sand_pictures') 
        print("Processing:", image_path)

        image = cv2.imread(image_path)








h, w = grayscale(image).shape 

margin_y = h // 20
margin_x = w // 3

start_y = margin_y
end_y = h - margin_y
start_x = margin_x
end_x = w - margin_x

cropped = gray[start_y:end_y, start_x:end_x]

cv2.imshow("Cropped left-right", cropped)
cv2.waitKey(0)
cv2.destroyAllWindows()
