#!/usr/bin/env python
'''
This code converts a high-resolution picture to a low resolution black/white image for path generation
Supported image formats are .JPEG2000, .JPEG, .png, .tiff, .bmp
'''
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

DARK_THRE = 100

def get_pic_array(PIC_FILE_PATH):
    img = Image.open(PIC_FILE_PATH).convert('L')
    original_size = img.size
    y_pixel_num = 80
    x_pixel_num = int(y_pixel_num*float(original_size[0])/float(original_size[1]))
    img = img.resize((x_pixel_num,y_pixel_num),Image.ANTIALIAS)
    p = np.array(img)
    for row in range(len(p)):
        for cln in range(len(p[0])):
            if p[row][cln] > DARK_THRE:
                p[row][cln] = 255
            else:
                p[row][cln] = 1
    return p

if __name__ == '__main__':
    p = get_pic_array("../image/sawyer_image.jpg")
    print "p size: ", p.shape
    plt.imshow(p, cmap='gray', vmin=0, vmax=255)
    plt.show()
