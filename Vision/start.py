import cv2
import numpy as np
import matplotlib.pyplot as plt
import os


def grayscale(image):

    image = cv2.imread('sand_pictures/sand5.jpg')

    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 



def hist_eq(img):

    equ = cv2.equalizeHist(img)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4,4))
    cl1 = clahe.apply(img)

    res = np.hstack((img, equ, cl1))

    #cv2.imshow('Original | Equalized | CLAHE', res)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

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

    return {
        "global": th_global,
        "adaptive_mean": th_adapt_mean,
        "adaptive_gaussian": th_adapt_gauss,
        "otsu": th_otsu,
        "otsu_blur": th_otsu_blur,
    }






#thresholds(hist_eq(grayscale('/home/marilene/github/Biomimicry_group6/Vision/sand_pictures/sand5.jpg')))


#### grain size (mm)= distance to sand in mm x grain size in pixels / focal length in pixels