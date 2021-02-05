import cv2
import imutils
import numpy as np
from math import cos, sin, radians
import time


class LidarSensorSimple():
    def __init__(self):

        # goc quet cua lidar (-angle ... 0 ... angle)
        self.angle = 90

        # met to centimet
        self.centimet = lambda m: int(m*100)  # (cm)

        # pham vi quet
        self.rangeDetect = 3

        # image_mapping_values
        self.mapSize = 3
        self.image_size = [self.centimet(self.rangeDetect), self.centimet(self.rangeDetect)]
        self.W, self.H, self.cW, self.cH = self.image_size[1], self.image_size[0], self.image_size[1]//2, self.image_size[0]//2

        # dangerZone
        self.duoixe = 13 #duoi xe
        self.disDanger = 170  # khoang cach nguy hiem (cm)
        self.before_threshold = self.cH - ((self.cH*self.disDanger)//self.centimet(self.rangeDetect)) #[:H-before_threshold] quy doi khoang cach px
        self.behind_threshold = 20

        # vatcan song song voi xe:
        self.nextto_threshold = 20
        self.rangeHasObj = 20
        self.leftDanger, self.rightDanger = self.cW - self.nextto_threshold, self.cW + self.nextto_threshold
        
        self.roiW = self.rightDanger - self.leftDanger
        self.roiH = ((self.cH + self.duoixe)-self.before_threshold)
        
        #khoanh_vung_xe:
        self.carWidth = 15
        self.carRanges = lambda w: int(w/(self.rangeDetect*self.carWidth))
        self.minLeft, self.maxLeft = 0, (self.roiW//2)-self.carRanges(self.W)
        self.minRight, self.maxRight = (self.roiW//2) + self.carRanges(self.W), self.centimet(self.rangeDetect)-1
        self.carPosX, self.carPosY = self.roiW//2, (self.roiH//2) + self.duoixe

###############################
    def dangerArea(self, ob_fw, ob_bh, ob_side):
        isObstacle = 'none'
        if len(ob_fw) > 0:
            for cnt, d in ob_fw:
                x1, x2, y1, y2 = cnt
                distance = d
                if (0 < distance <= self.disDanger):
                    isObstacle = 'forware'
                    print(time.time(),"Lite: ",isObstacle)
                    return isObstacle
                else:
                    isObstacle = 'none'
        if len(ob_bh) > 0:
            for cnt, d in ob_bh:
                x1, x2, y1, y2 = cnt
                distance = d
                if (0 < distance <= self.disDanger):
                    isObstacle = 'behind'
                    print(time.time(),"Lite: ",isObstacle)
                    return isObstacle
                else:
                    isObstacle = 'none'
        if len(ob_side) > 0:
            for cnt, d in ob_side:
                x1, x2, y1, y2 = cnt
                distance = d
                if (0 < distance <= self.disDanger):
                    isObstacle = 'side'
                    print(time.time(),"Lite: ",isObstacle)
                    return isObstacle
                else:
                    isObstacle = 'none'
        # print(self.carWidth)
        # print("Lite: ",isObstacle)
        return isObstacle

    # lay gia tri khoang cach
    def getRanges(self, point, pointRanges):

        x1, x2, y1, y2 = point

        for x, y, d in pointRanges:
            if x1 < x < x2 and y1 < y < y2:
                return d

    def detectObject(self, arrayRanges):

        img, pointsRange = self.lidar2CV(arrayRanges)

        forware_object = []
        behind_object = []
        side_object = []
        contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        if len(contours) > 0:
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                x1, x2, y1, y2 = x, x+w, y, y+h
                temp = [x1, x2, y1, y2]
                d = self.getRanges(temp, pointsRange)

                if ((x1 >= self.maxLeft and x2 <= self.minRight) 
                        or (x1 <= self.maxLeft and x2 >= self.roiW//2) 
                        or (x1 <= self.roiW//2 and x2 >= self.minRight) 
                        or (x1 <= self.maxLeft and x2 >= self.maxLeft) 
                        or (x1 <= self.minRight and x2 >= self.minRight)) and (y2 < (self.carPosY-self.rangeHasObj)):
                    forware_object.append([temp,d])
                if ((self.roiW//2) < x1 < self.minRight) and (y2 < (self.carPosY-self.rangeHasObj)):
                    forware_object.append([temp,d])
                if (y1 >= (self.carPosY + self.duoixe)):
                    behind_object.append([temp,d])
                if (y1 <= self.carPosY <= y2) or (y1 <= (self.carPosY-self.rangeHasObj) <= y2) or (y1 < (self.carPosY + self.duoixe) <= y2):
                    side_object.append([temp,d])

        return forware_object, behind_object, side_object, img

    # ket noi va xoa bo cac diem khong can thiet
    def pointsConnected(self, img):

        imgs = cv2.GaussianBlur(img, (13, 13), 0)

        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            img, None, None, None, 8, cv2.CV_32S)

        areas = stats[1:, cv2.CC_STAT_AREA]

        result = np.zeros((labels.shape), np.uint8)

        for i in range(0, nlabels - 1):
            if areas[i] >= 100:
                result[labels == i + 1] = 255
        return result

    # draw
    def drawable(self, img):
        
        cv2.line(img,(0,self.carPosY),(self.roiW,self.carPosY),255,1)
        # cv2.line(img,(0,((self.roiH//2) + self.behind_threshold)-20),(self.roiW,((self.roiH//2) + self.behind_threshold-20)),255,1)

        cv2.line(img,(self.roiW//2,0),(self.roiW//2,(self.carPosY + self.duoixe)),255,1)
        cv2.line(img,(self.maxLeft,0),(self.maxLeft,(self.carPosY + self.duoixe)),255,1)
        cv2.line(img,(self.minRight,0),(self.minRight,(self.carPosY + self.duoixe)),255,1)

        return img

    # mapping
    def lidar2CV(self, arrayRanges):

        maps = np.zeros((self.roiH + self.behind_threshold,self.roiW), dtype=np.uint8)
        points = []

        for i in range(0, len(arrayRanges)):
            if arrayRanges[i]/100 < self.rangeDetect:
                yy = cos(radians(i-self.angle)) * arrayRanges[i]/100*(self.cH/self.mapSize)
                xx = sin(radians(i-self.angle)) * arrayRanges[i]/100*(self.cW/self.mapSize) # reverge : sin(radians(180+(i-angle)))

                x = self.roiW//2 + xx
                y = (self.carPosY) - yy

                cv2.circle(maps, (int(x), int(y)), self.W//self.H + 3, 255, -1)

                points.append([int(x), int(y), arrayRanges[i]])

        return self.pointsConnected(maps), points
