#!/usr/bin/env python
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

DARK_THRE = 100

def get_pic_array(PIC_FILE_PATH):
    img = Image.open(PIC_FILE_PATH).convert('L')
    original_size = img.size
    y_pixel_num = 200
    x_pixel_num = int(y_pixel_num*float(original_size[0])/float(original_size[1]))
    img = img.resize((x_pixel_num,y_pixel_num),Image.ANTIALIAS)
    # img_o.save("test-400.png", dpi=(100,200))
    # img = Image.open("test-400.png").convert('L')
    p = np.array(img)
    for row in range(len(p)):
        for cln in range(len(p[0])):
            if p[row][cln] > DARK_THRE:
                p[row][cln] = 255
            else:
                p[row][cln] = 1
    # img.show()
    # image = Image.fromarray(p,'L')
    # image.show()

    # plt.imshow(p, cmap='gray', vmin=0, vmax=255)
    # plt.show()
    print "inside pic:", p.shape
    return p

get_pic_array("../images/IMG_4926_adjust_2.jpg")

