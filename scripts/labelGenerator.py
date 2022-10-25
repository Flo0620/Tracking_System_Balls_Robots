"""
This script can be used to generate the test.txt and train.txt files for a dataset, which define which images are used for training and which for validation.
Furthermore, it creates the label txt file for each image from a txt file that contains the annotations of all images.
"""
import os

generateTestTrain = False
AddImagesWithoutBall =True

f = open("annotations600.txt","r")#the file which contains the annotations for the images
imgs=set()
anteilTest=10 #in Prozent
counterimgs=0
imgpath = "data/custom/images/"

#this part creates the label txt file for an image with depicted balls
lines = f.readlines()
for line in lines:
    imgnamepng = line.split(":")[0]
    imgnametxt= line.split(":")[0][0:-4]+".txt"
    flabel = open("labels1200/"+imgnametxt,"a")
    flabel.write(line.split(":")[1])
    flabel.close()
    if generateTestTrain == True:
        imgs.add(imgnamepng)


def get_number(count):
        end = str(count)
        string=""
        while count/10000<1:
            string=string+"0"
            count = count*10
        return string+end

if AddImagesWithoutBall:
    for i in range(604,1209):
        imgs.add("img"+str(get_number(i))+".png")
        print(str(i))
        flabel = open("labels1200/"+"img"+str(get_number(i))+".txt","a")

if generateTestTrain == True:
    ftest=open("test.txt","a")
    ftrain=open("train.txt","a")
    for img in imgs:
        if counterimgs%int(100/anteilTest)==0:
            ftest.write(imgpath+img+"\n")
        else:
            ftrain.write(imgpath+img+"\n")
        counterimgs+=1
    ftrain.close()
    ftest.close()
f.close()
