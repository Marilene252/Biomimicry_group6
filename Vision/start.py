import cv2
import numpy as np
import matplotlib.pyplot as plt


def grayscale(image):

    image = cv2.imread('sand_pictures/sand5.jpg')

    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 



def hist_eq(img):
    'resource used: https://docs.opencv.org/4.x/d5/daf/tutorial_histogram_equalization.html'

    equ = cv2.equalizeHist(img)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4,4))
    cl1 = clahe.apply(img)

    res = np.hstack((img, equ, cl1))

    return res


def thresholds(gray_img, use_blur=False, show_hist=True):
    'resource used: https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html'

    # blur preprocessing
    if use_blur:
        img_med = cv2.medianBlur(gray_img, 5)
        img_gauss = cv2.GaussianBlur(gray_img, (5, 5), 0)
    else:
        img_med = gray_img.copy()
        img_gauss = gray_img.copy()

    # 1. Global Threshold
    _, th_global = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY)

    # 2. Adaptive Mean
    th_adapt_mean = cv2.adaptiveThreshold(
        gray_img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

    # 3. Adaptive Gaussian
    th_adapt_gauss = cv2.adaptiveThreshold(
        gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # 4. Otsu’s on raw image
    _, th_otsu = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 5. Otsu’s after Gaussian blur 
    _, th_otsu_blur = cv2.threshold(img_gauss, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)


    titles = [
        "Original (CLAHE Input)",
        "Global Threshold (127)",
        "Adaptive Mean",
        "Adaptive Gaussian",
        "Otsu's (Raw)",
        "Otsu's (Blurred)" if use_blur else "Otsu's (Same Image)"
    ]
    images = [gray_img, th_global, th_adapt_mean, th_adapt_gauss, th_otsu, th_otsu_blur]

    plt.figure(figsize=(12, 8))
    for i in range(6):
        plt.subplot(2, 3, i + 1)
        plt.imshow(images[i], cmap="gray", vmin=0, vmax=255)
        plt.title(titles[i])
        plt.xticks([]), plt.yticks([])
    plt.tight_layout()
    plt.show()

    if show_hist:
        plt.figure(figsize=(10, 4))
        plt.subplot(1, 2, 1)
        plt.hist(gray_img.ravel(), 256)
        plt.title("Histogram (Input)")
        plt.subplot(1, 2, 2)
        plt.imshow(th_otsu, cmap="gray")
        plt.title("Otsu’s Result")
        plt.xticks([]), plt.yticks([])
        plt.tight_layout()
        plt.show()

    return {"global": th_global, "adaptive_mean": th_adapt_mean, "adaptive_gaussian": th_adapt_gauss, "otsu": th_otsu, "otsu_blur": th_otsu_blur,}


def morphological_cleanup(image, kernel_size=3, open_iter=1, close_iter=1):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    cleaned1 = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel, iterations=open_iter)

    cleaned = cv2.morphologyEx(cleaned1, cv2.MORPH_CLOSE, kernel, iterations=close_iter)

    return cleaned


def create_markers(img):

    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=2)

    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist_transform, 0.2 * dist_transform.max(), 255, 0)
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
    segmented[markers == -1] = [0, 0, 255]

    return segmented, markers, sure_fg, sure_bg, unknown
           
           
#def extract_contours(): 

#def compute_grain_size():



def show_full_segmentation(path):
    """
    Runs the full sand segmentation pipeline and displays results.
    """
    # 1. Load and preprocess
    gray = grayscale(path)
    eq = cv2.equalizeHist(gray)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4,4))
    clahe_img = clahe.apply(gray)

    # 2. Threshold + morph cleanup
    th = thresholds(clahe_img)['otsu']  # pick one (adaptive_mean or otsu)
    cleaned = morphological_cleanup(th)

    # 3. Watershed
    segmented, markers, sure_fg, sure_bg, unknown = apply_watershed(
        cv2.imread(path), cleaned
    )

    # 4. Show everything
    plt.figure(figsize=(12, 10))
    titles = ["Original", "CLAHE + Otsu Threshold", "After Morph Cleanup", 
              "Sure Foreground", "Sure Background", "Unknown Region", "Watershed Segmentation"]
    images = [gray, th, cleaned, sure_fg, sure_bg, unknown, segmented]

    for i, (img, title) in enumerate(zip(images, titles)):
        plt.subplot(3, 3, i + 1)
        if len(img.shape) == 2:
            plt.imshow(img, cmap='gray')
        else:
            plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.title(title)
        plt.axis('off')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    path = '/home/marilene/github/Biomimicry_group6/Vision/sand_pictures/sand1.jpg'
    show_full_segmentation(path)




#thresholds(hist_eq(grayscale('/home/marilene/github/Biomimicry_group6/Vision/sand_pictures/sand5.jpg')))


#### grain size (mm)= distance to sand in mm x grain size in pixels / focal length in pixels