import cv2
import os,shutil
from matplotlib import pyplot as plt
import numpy as np

class Mouse:
    def __init__(self,img):
        self.clear()
        self.color=(0,0,0)
        self.img=img
    def draw_circle(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.flag==0:
                self.flag+=1 
                self.xy1=(x,y)
                print(self.xy1,self.xy2)
                return 0
            if self.flag==1:
                self.xy2=(x,y)
                print(self.xy1,self.xy2)
                cv2.line(self.img,self.xy1,self.xy2,self.color,2,8)
                self.clear()
                return 0
        if event == cv2.EVENT_RBUTTONDOWN:
            print("clear click")
            self.clear()
        if event == cv2.EVENT_MBUTTONDBLCLK:
            if self.color==(0,0,0):
                self.color=(255,255,255)
                print("change color to",self.color)
                return 0
            if self.color==(255,255,255):
                self.color=(0,0,0)
                print("change color to",self.color)
                return 0
    def clear(self):
        self.xy1=[]
        self.xy2=[]
        self.flag=0
        
###1，加载图片
filepath = '/home/lc/2022HPX/new_map.png'  ###图像路径，注意：这里的路径不能包含有中文名
img = cv2.imread(filepath)
n1=img[682:986,999:1562,:]

b=n1[:,:,0]
g=n1[:,:,1]
r=n1[:,:,2]
xy=np.where(n1[:,:,0]<3)
n1[xy[0],xy[1],:]=0
xy=np.where(n1[:,:,0]>0)
n1[xy[0],xy[1],:]=255
cv2.namedWindow('image')
a=Mouse(n1)
def draw_circle(event,x,y,flags,param):
    a.draw_circle(event,x,y,flags,param)
cv2.setMouseCallback('image',draw_circle)
while(1):
    cv2.imshow('image',a.img)
    img[682:986,999:1562,:]=a.img
    cv2.imwrite('new_map.png',img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    if cv2.waitKey(20) & 0xFF == 27:
        break
cv2.destroyAllWindows()