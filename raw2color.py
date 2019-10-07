import glob
import os
import tqdm
import cv2


path = 'E:/Datasets/NKBVS/Taganrog/2019-08-16-11-24-24_0_a/vis/left/image_raw/data'

path_expression = path + '/*.png'
imgs = glob.glob(path_expression, recursive=True)

print(imgs[0])

converted_imgs = []
for im in imgs:
    converted_imgs.append(im.replace(r'image_raw/data', r'image_raw/converted'))
print(converted_imgs[0])

def getImage(im_path):
    if os.path.isfile(im_path):
        img = cv2.imread(im_path, cv2.IMREAD_UNCHANGED)
        img = cv2.cvtColor(img, cv2.COLOR_BAYER_BG2BGR)
        #img = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2RGB)
        #img = img[:, :, [2, 1, 0]]
        return img
    else:
        return None


for i in tqdm.tqdm(range(len(converted_imgs))):
    if not os.path.exists(os.path.dirname(converted_imgs[i])):
        os.makedirs(os.path.dirname(converted_imgs[i]))
    if not os.path.exists(converted_imgs[i]):
        try:
            color_image = getImage(imgs[i])
            cv2.imwrite(converted_imgs[i], color_image)
        except:
            print(imgs[i])