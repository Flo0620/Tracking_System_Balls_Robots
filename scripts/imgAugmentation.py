"""
This script augments all png images in a given directory. 
For each image a txt file with the same name has to be given in which the labels for the image are provided.
The images are mirrored horizontally and vertically and they are lightend and darkend by a constant.
By that for each input image 11 augmented images are generated.
"""
import os
import cv2

directory = '1600Dataset'

def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def decrease_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = value
    v[v < lim] = 0
    v[v >= lim] -= value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def augment_brightness(img,filename,boxes):
    imgb1 = increase_brightness(img,40)
    cv2.imwrite(os.path.join(directory,filename[0:-4]+"b1.png"),imgb1)
    txtfile=open(os.path.join(directory,filename[0:-4]+"b1.txt"),'w')
    txtfile.write(boxes)
    txtfile.close()
    imgb2 = decrease_brightness(img,40)
    cv2.imwrite(os.path.join(directory,filename[0:-4]+"b2.png"),imgb2)
    txtfile=open(os.path.join(directory,filename[0:-4]+"b2.txt"),'w')
    txtfile.write(boxes)
    txtfile.close()

for filename in os.listdir(directory):
    if filename.endswith("png"):
        imgpath=os.path.join(directory,filename)
        img = cv2.imread(imgpath)
        #cv2.imwrite(os.path.join(directory,filename[0:-4]+".png"),img)
        boxesfile = open(os.path.join(directory,filename[0:-4]+".txt"),'r')
        boxlines=boxesfile.readlines()
        boxes=""
        boxesh=""
        boxesv=""
        boxeshv=""
        for box in boxlines:
            n,x,y,w,h=box.split(" ")
            x=float(x)
            y=float(y)
            boxes=boxes+n+" "+str(x)+" "+str(y)+" "+w+" "+h
            boxesh=boxesh+n+" "+str(1-x)+" "+str(y)+" "+w+" "+h
            boxesv=boxesv+n+" "+str(x)+" "+str(1-y)+" "+w+" "+h
            boxeshv=boxeshv+n+" "+str(1-x)+" "+str(1-y)+" "+w+" "+h

        boxesfile.close()

        augment_brightness(img,filename[0:-4]+".png",boxes)
        imgh= cv2.flip(img,1)
        cv2.imwrite(os.path.join(directory,filename[0:-4]+"h.png"),imgh)
        txtfile=open(os.path.join(directory,filename[0:-4]+"h.txt"),'w')
        txtfile.write(boxesh)
        txtfile.close()
        augment_brightness(imgh,filename[0:-4]+"h.png",boxesh)

        imgv= cv2.flip(img,0)
        txtfile=open(os.path.join(directory,filename[0:-4]+"v.txt"),'w')
        txtfile.write(boxesv)
        txtfile.close()
        cv2.imwrite(os.path.join(directory,filename[0:-4]+"v.png"),imgv)
        augment_brightness(imgv,filename[0:-4]+"v.png",boxesv)

        imghv= cv2.flip(img,-1)
        cv2.imwrite(os.path.join(directory,filename[0:-4]+"hv.png"),imghv)
        txtfile=open(os.path.join(directory,filename[0:-4]+"hv.txt"),'w')
        txtfile.write(boxeshv)
        txtfile.close()
        augment_brightness(imghv,filename[0:-4]+"hv.png",boxeshv)
        


