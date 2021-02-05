import cv2
import imutils
import numpy as np
from math import cos, sin, radians

class LidarSensor():
    def __init__(self):

        #goc quet cua lidar (-angle ... 0 ... angle)
        self.angle = 90

        ######### met to centimet
        self.centimet = lambda m: int(m*100) #(cm)

        #pham vi quet
        self.rangeDetect = 3

        #image_mapping_values
        self.mapSize = 3
        self.image_size = [self.centimet(self.rangeDetect)//2,self.centimet(self.rangeDetect)]
        self.W, self.H, self.cW, self.cH = self.image_size[1], self.image_size[0], self.image_size[1]//2, self.image_size[0]//2

        #dangerZone
        self.disDanger = 170 #khoang cach nguy hiem (cm)
        self.pxDanger = self.H - ((self.H*self.disDanger)//self.centimet(self.rangeDetect)) #[:H-pxDanger] quy doi khoang cach px

        self.carWidth = 15
        self.carRanges = lambda w: int(w/(self.rangeDetect*self.carWidth))
        self.minLeft, self.maxLeft = 0, self.cW-self.carRanges(self.W)
        self.minRight, self.maxRight = self.cW+self.carRanges(self.W), self.centimet(self.rangeDetect)-1

        #vatcan song song voi xe:
        self.vatcan_horizontal = 20
        self.rangeHasObj = 130
        self.leftDanger, self.rightDanger = self.maxLeft - self.vatcan_horizontal, self.minRight + self.vatcan_horizontal

    def dangerArea(self,obj):
        isObstacle = False
        if len(obj) > 0:
            for cnt, d in obj:
                x1, x2, y1, y2 = cnt
                distance = d
                if (0 < distance <= self.disDanger) or (y2 > self.pxDanger) or ((self.minRight < x1 < self.rightDanger) and (y2 > self.rangeHasObj)):
                    print("Nguy hiem",distance)
                    isObstacle = True
                    return isObstacle
                else:
                    print('canh bao')
                    isObstacle = False
        else:
            print('an toan')
        
        return isObstacle

    #lay gia tri khoang cach
    def getRanges(self,point,pointRanges):

        x1, x2, y1, y2 = point

        for x,y,d in pointRanges:
            if x1<x<x2 and y1<y<y2:
                return d

    def detectObject(self,arrayRanges):

        img, pointsRange = self.lidar2CV(arrayRanges)

        l_object, c_object, r_object = [], [], []
        contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        if len(contours) > 0:
            for cnt in contours:
                x,y,w,h = cv2.boundingRect(cnt)
                x1, x2, y1, y2 = x, x+w, y, y+h
                temp = [x1, x2, y1, y2]

                if x1 < self.maxLeft and x2 < self.maxLeft:
                    d = self.getRanges(temp,pointsRange)
                    l_object.append([temp,d])
                if x1 > self.minRight and x2 > self.minRight:
                    d = self.getRanges(temp,pointsRange)
                    r_object.append([temp,d])
                if (x1 > self.maxLeft and x2 < self.minRight) or (x1 < self.maxLeft and x2 > self.cW) or (x1 < self.cW and x2 > self.minRight) or (x1 < self.maxLeft and x2 > self.maxLeft) or (x1 < self.minRight and x2 > self.minRight):
                    d = self.getRanges(temp,pointsRange)
                    c_object.append([temp,d])
                if (self.minRight < x1 < self.rightDanger) and (y2 > self.rangeHasObj):
                    c_object.append([temp,d])

        return l_object, c_object, r_object, img

    #ket noi va xoa bo cac diem khong can thiet
    def pointsConnected(self,img):

        imgs = cv2.GaussianBlur(img,(13,13),0)

        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, None, None, None, 8, cv2.CV_32S)

        areas = stats[1:,cv2.CC_STAT_AREA]

        result = np.zeros((labels.shape), np.uint8)

        for i in range(0, nlabels - 1):
            if areas[i] >= 100:
                result[labels == i + 1] = 255
        return result

    #draw 
    def drawable(self,img):
        cv2.line(img,(self.minRight,0),(self.minRight,self.H),255,1)
        cv2.line(img,(self.maxLeft,0),(self.maxLeft,self.H),255,1)
        cv2.line(img,(self.cW,0),(self.cW,self.H),255,1)
        cv2.circle(img,(self.cW,self.H),self.H,255,1)

        cv2.line(img,(self.rightDanger,130),(self.rightDanger,self.H),255,1)
        cv2.line(img,(self.leftDanger,130),(self.leftDanger,self.H),255,1)

        return img

    #mapping
    def lidar2CV(self,arrayRanges):

        maps = np.zeros((self.H,self.W), dtype=np.uint8)
        points = []

        for i in range(0,len(arrayRanges)):
            if arrayRanges[i]/100 < self.rangeDetect:
                yy = cos(radians(i-self.angle)) * arrayRanges[i]/100*(self.H/self.mapSize)
                xx = sin(radians(i-self.angle)) * arrayRanges[i]/100*(self.cW/self.mapSize) #reverge : sin(radians(180+(i-angle)))
                
                x = self.cW + xx
                y = self.H - yy

                points.append([int(x),int(y),arrayRanges[i]])
                cv2.circle(maps,(int(x),int(y)),self.W//self.H + 3,255,-1)

        return self.pointsConnected(maps),points