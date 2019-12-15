import glob
import os
import tqdm
import cv2
import argparse

def getImage(im_path):
    if os.path.isfile(im_path):
        img = cv2.imread(im_path, cv2.IMREAD_UNCHANGED)
        #img = cv2.cvtColor(img, cv2.COLOR_BAYER_BG2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2RGB)
        #img = img[:, :, [2, 1, 0]]
        return img
    else:
        return None

def main_raw2color(path):

    #path = 'E:/Datasets/NKBVS/NAMI-Polygon/2019-01-31-15-42-51_kia_velo_gps_time/stereo/left/image_raw/data'

    path_expression = path + '/*.png'
    imgs = glob.glob(path_expression)

    print('Path processing is started..')

    converted_imgs = []
    for im in imgs:
        converted_imgs.append(im.replace(r'image_raw/data', r'image_raw/converted'))
    #print(converted_imgs[0])

    for i in tqdm.tqdm(range(len(converted_imgs))):
        if not os.path.exists(os.path.dirname(converted_imgs[i])):
            os.makedirs(os.path.dirname(converted_imgs[i]))
        if not os.path.exists(converted_imgs[i]):
            try:
                color_image = getImage(imgs[i])
                cv2.imwrite(converted_imgs[i], color_image)
            except:
                print(imgs[i])

if __name__ == '__main__':
	argparser = argparse.ArgumentParser(description='Convert image form Bayer to RGB')
	argparser.add_argument('path', help='path to bag file')
	args = argparser.parse_args()
	main_raw2color(**vars(args))
