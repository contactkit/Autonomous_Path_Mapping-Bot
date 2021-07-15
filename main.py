import gym
import numpy as np
import pix_main_arena
import time

import pybullet as p
import pybullet_data
import cv2
import cv2.aruco as aruco
import numpy.linalg as la
s=12
nodes = np.full((s,s),0,np.uint8)
k=0
for i in range(s):
    for j in range(s):
        nodes[i][j]=k
        k=k+1

pos = np.full((s,s),0,np.int16)


def ssort(x):
    av=np.full((3,1,2),0,np.int16)
    if(x[0][0][0]<=x[1][0][0] and x[0][0][0]<=x[2][0][0]):
        if(x[2][0][0]<=x[1][0][0]):
            av[0][0]=x[0][0]
            av[1][0]=x[2][0]
            av[2][0]=x[1][0]
        else:
            av[0][0]=x[0][0]
            av[1][0]=x[1][0]
            av[2][0]=x[2][0]

    elif(x[0][0][0]<=x[1][0][0] and x[0][0][0]>=x[2][0][0]):
        
        av[0][0]=x[2][0]
        av[1][0]=x[0][0]
        av[2][0]=x[1][0]
    elif(x[0][0][0]<=x[2][0][0] and x[0][0][0]>=x[1][0][0]):
        
        av[0][0]=x[1][0]
        av[1][0]=x[0][0]
        av[2][0]=x[2][0]
    elif(x[0][0][0]>=x[1][0][0] and x[0][0][0]>=x[2][0][0]):
        if(x[2][0][0]<=x[1][0][0]):
            av[0][0]=x[2][0]
            av[1][0]=x[1][0]
            av[2][0]=x[0][0]
        elif(x[2][0][0]>=x[1][0][0]):
            av[0][0]=x[1][0]
            av[1][0]=x[2][0]
            av[2][0]=x[0][0]

    
    return av
  
class Graph():
  
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)] 
                    for row in range(vertices)]
  
    def printSolution(self, dist,cell):
        print ("Vertex \tDistance from Source \tcell")
        for node in range(self.V):
            print (node, "\t", dist[node], "\t", cell[node])
  
    def minDistance(self, dist, sptSet):
  
     
        min = 1000
        min_index=-1
       
        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
  
        return min_index
  
    def dijkstra(self, src):
        
        cell=[0]*self.V
        dist = [1000] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
  
        for cout in range(self.V):
  
            u = self.minDistance(dist, sptSet)
  
            sptSet[u] = True
  
            for v in range(self.V):
                if self.graph[u][v] > 0 and self.graph[u][v] and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]:
                        dist[v] = dist[u] + self.graph[u][v]                       
                        cell[v]=cell[u]+1
  
        self.printSolution(dist,cell)
        return dist,cell
  
def shortest_path(node, dist,src,s,cell):
    path=[]
    #path.append(node)
    d = np.full(4,1000,np.int16)
    while (node!=src):
        d = np.full(4,1000,np.int16)
        if(int(node/s)!=0 and g.graph[node-s][node]>0):
            d[0]=dist[node-s]
        if(int(node/s)!=s-1 and g.graph[node+s][node]>0):
            d[1]=dist[node+s]
        if(node%s!=0 and g.graph[node-1][node]>0):
            d[2]=dist[node-1]
        if(node%s!=s-1 and g.graph[node+1][node]>0):
            d[3]=dist[node+1]
        
        k=min(d)
        i=0
        a = []
        if(k==d[0]):
            a.append(node-s)
            i=i+1
        if(k==d[1]):
            a.append(node+s)
            i=i+1
        if(k==d[2]):
            a.append(node-1)
            i=i+1
        if(k==d[3]):
            a.append(node+1)
            i=i+1
        mincell=cell[a[0]]
        mind=0
        #print(a)
        for j in range(i):
            
            u=a[j]
            if(cell[u]<=mincell):
                mincell=cell[a[j]]
                mind=j
                
        node=a[mind]
        path.append(node)
    return path

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle(vector1, vector2):
    v1_u = unit_vector(vector1)
    v2_u = unit_vector(vector2)
    minor = np.linalg.det(
        np.stack((v1_u[-2:], v2_u[-2:]))
    )

    return np.sign(minor) * np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
def py_agl(v1, v2):
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)
def final_angle(v1,v2):
    angle1=np.degrees(angle(v1,v2))
    angle2=np.degrees(py_agl(v1,v2))
    if angle1==0:
        return angle2
    else:
        return angle1
def align(v1,v2):
    
    agl=final_angle(v1,v2)
    if agl<=-100:
        for i in range(140):
            p.stepSimulation()
            env.move_husky(27, -27, 27, -27)
            p.stepSimulation()
            env.move_husky(5, -5, 5, -5)
        for i in range(10):
            p.stepSimulation()
            env.move_husky(-5, -5, 0, 0)
    if agl<=180 and agl>=100:
        for i in range(140):
            p.stepSimulation()
            env.move_husky(-27, 27, -27, 27)
            p.stepSimulation()
            env.move_husky(-5, 5, -5, 5)
        for i in range(10):
            p.stepSimulation()
            env.move_husky(-5, -5, 0, 0)
        # print('R')
    elif agl<0:
        p.stepSimulation()
        env.move_husky(7, -7, 7, -7)
        p.stepSimulation()
        env.move_husky(5, -5, 5, -5)
        # p.stepSimulation()
        # env.move_husky(-0.5, -0.5, 0, 0)
        #print('R')
    elif agl>0:
        p.stepSimulation()
        env.move_husky(-7, 7, -7, 7)
        p.stepSimulation()
        env.move_husky(-5, 5, -5, 5)
        # p.stepSimulation()
        # env.move_husky(-0.5,-0.5, 0, 0)
        #print('L')
  
       
def move(pt):
    while pt:
        print('next node')
        dt=pt.pop()
        
        print(dt)
        img1=env.camera_feed()
        
        c,n,v1,x1,y1=bot_pos(img1)
        
        while v1==-1:
            print('x')
            p.stepSimulation()
            env.move_husky(20,-20,20,-20)
            img1=env.camera_feed()
            
            c,n,v1,x1,y1=bot_pos(img1)
        x2=(dt%s)*x+x//2
        y2=(int(dt/s))*y+y//2
        if(abs(y1-y2)<17):
            v2=[(x2-x1),0]
        elif(abs(x2-x1)<17):
            v2=[0,(y1-y2)]
        else:
            v2=[(x2-x1),(y1-y2)]

        ag2=np.degrees(py_agl(v1,v2))
        
    
        
        while(c!=dt):
            
            
            while ag2>2:  
                #print(ag2)                 
                p.stepSimulation()
                align(v1,v2)
                img1=env.camera_feed()
                

                c,n,v1,x1,y1=bot_pos(img1)
                while v1==-1:
                    p.stepSimulation()
                    env.move_husky(20,-20,20,-20)
                    img1=env.camera_feed()
                    
                    c,n,v1,x1,y1=bot_pos(img1)
                if(abs(y1-y2)<17):
                    v2=[(x2-x1),0]
                elif(abs(x2-x1)<17):
                    v2=[0,(y1-y2)]
                else:
                    v2=[(x2-x1),(y1-y2)]
                ag2=np.degrees(py_agl(v1,v2))
                #fa=final_angle(v1,v2)
            if ag2<2:
               
                
                dist=abs(abs(v2[0])+abs(v2[1])-14)
                
                #print(dist)
                if(dist>(x-x/40)):
                    p.stepSimulation()
                    env.move_husky(35,35,35,35)
                else:
                    p.stepSimulation()
                    env.move_husky(10,10,10,10)
                    p.stepSimulation()
                    env.move_husky(10,10,10,10)
            img1=env.camera_feed()
            
            
            c,n,v1,x1,y1=bot_pos(img1)
            while v1==-1:
                #print('l')
                p.stepSimulation()
                env.move_husky(20,-20,20,-20)
                img1=env.camera_feed()
                
                c,n,v1,x1,y1=bot_pos(img1)
            if(abs(y1-y2)<17):
                v2=[(x2-x1),0]
            elif(abs(x2-x1)<17):
                v2=[0,(y1-y2)]
            else:
                v2=[(x2-x1),(y1-y2)]
            ag2=np.degrees(py_agl(v1,v2))

        
    
        for i in range(5):
            p.stepSimulation()
            env.move_husky(12,12,12,12)
            img1=env.camera_feed()
        
    for i in range(15):
        p.stepSimulation()
        env.move_husky(12,12,12,12)
        img1=env.camera_feed()
    for i in range(20):
        p.stepSimulation()
        env.move_husky(0,0,0,0)
        img1=env.camera_feed()
    
        img1=env.camera_feed()
    img1=env.camera_feed()
def read_des():
    w=0
    im = env.camera_feed()
    img2 = im[int(r2[1]):int(r2[1]+r2[3]), int(r2[0]):int(r2[0]+r2[2])]
    hsv=cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
    mask_blue=cv2.inRange(hsv,lv_blue,uv_blue)
    res_blue = cv2.bitwise_and(img2, img2, mask=mask_blue)
    res_blue[np.where((res_blue==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_b, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_b:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        
        
        if area > 400:       
            #print("/////")
            #print(pnode[m])
            #print(nodes[int(ceny/y)][int(cenx/x)])         
            if(len(approx)==4):
                #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                if(nodes[int(ceny/y)][int(cenx/x)]==pnode[m]):
                    w=0
                
                
            elif(len(approx)>5):
                #print("+++++")
                #print(nodes[int(ceny/y)][int(cenx/x)])  
                if(nodes[int(ceny/y)][int(cenx/x)]==pnode[m]):
                    w=1
    return w
def bot_pos(im2):
    im2 = im2.copy()
   # print("----")
    #print(img2.shape)
    gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
   
    corners=np.asarray(corners)
    im2 = aruco.drawDetectedMarkers(im2, corners, borderColor=(0, 0, 255))
    #cv2.imshow('img',img)
    #print(ids)
    if ids is not None:
        x1=(corners[0][0][0][0]+corners[0][0][1][0])/2-wx
        y1=(corners[0][0][0][1]+corners[0][0][1][1])/2-wy
        x2=(corners[0][0][2][0]+corners[0][0][3][0])/2-wx
        y2=(corners[0][0][2][1]+corners[0][0][3][1])/2-wy
       # print(x1," ",x2," ",y1," ",y2)
        X=(x1+x2)/2
        Y=(y1+y2)/2
        
        l=int((x1)/x)
        h=int(y1/y)
        L=int(X/x)
        H=int(Y/y)
        
        if l>s-1:
            l=s-1
        if h>s-1:
            h=s-1
        if L>s-1:
            L=s-1
        if H>s-1:
            H=s-1
        if l<0:
            l=0
        if h<0:
            h=0
        if L<0:
            L=0
        if H<0:
            H=0
        
        n=nodes[h][l]
        c=nodes[H][L]
        vect=[(x1-x2),(y2-y1)]
        return c,n,vect,X,Y
    return -1,-1,-1,-1,-1


    

if __name__=="__main__":
    env = gym.make("pix_main_arena-v0")
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    board = aruco.GridBoard_create(
        markersX=2,
        markersY=2,
        markerLength=0.09,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)
    #p.stepSimulation()    
    #env.remove_car()
    im = env.camera_feed()
    #cv2.imwrite("main.png",im)
    r2 = cv2.selectROI(im)
    img2 = im[int(r2[1]):int(r2[1]+r2[3]), int(r2[0]):int(r2[0]+r2[2])]
    wx=(im.shape[1]-img2.shape[1])/2
    wy=(im.shape[0]-img2.shape[0])/2
    hsv=cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
    x=img2.shape[0]/s
    y=img2.shape[1]/s
    pos[(s*s-1)//s][(s*s-1)%s]
    #print(wx," ",wy)
   
  
    pnode=np.full(2,-1,np.int16)
    m=0
    hnode=np.full(2,-1,np.int16)
    
    oneway=[]
    count=0
#---------FOR_RED--------------
    lv_red=np.array([0,52,52])
    uv_red=np.array([17,255,255])
    lv2_red=np.array([145,196,52])
    uv2_red=np.array([180,255,255])
    mask_red=cv2.inRange(hsv,lv_red,uv_red)
    mask2_red=cv2.inRange(hsv,lv2_red,uv2_red)
    res_red=cv2.addWeighted(mask_red,1,mask2_red,1,0)
    ct_r, _ = cv2.findContours(res_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_r:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:
            pos[int(ceny/y)][int(cenx/x)] = 4
            
           # cv2.drawContours(img2, cnt, -1, (0, 255, 0), 3)
           # cv2.putText(img2, "R", (cenx, ceny), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

   
#----------FOR__Yellow------------
    lv_yellow=np.array([12,52,50])
    uv_yellow=np.array([40,255,255])
    mask_yellow=cv2.inRange(hsv,lv_yellow,uv_yellow)
    res_yellow = cv2.bitwise_and(img2, img2, mask=mask_yellow)
    res_yellow[np.where((res_yellow==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_y, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_y:

        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:
            pos[int(ceny/y)][int(cenx/x)] = 3

 


#----------FOR__PINK------------
    lv_yellow=np.array([127,45,177])
    uv_yellow=np.array([180,142,255])
    mask_yellow=cv2.inRange(hsv,lv_yellow,uv_yellow)
    res_yellow = cv2.bitwise_and(img2, img2, mask=mask_yellow)
    res_yellow[np.where((res_yellow==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_y, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_y:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        approx=ssort(approx)
        #print(approx.shape)
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:

            pos[int(ceny/y)][int(cenx/x)]=250+1000
            pnode[m]=nodes[int(ceny/y)][int(cenx/x)]
            m=m+1
#----------FOR__GREEN------------
    lv_yellow=np.array([52,33,144])
    uv_yellow=np.array([72,255,255])
    mask_yellow=cv2.inRange(hsv,lv_yellow,uv_yellow)
    res_yellow = cv2.bitwise_and(img2, img2, mask=mask_yellow)
    res_yellow[np.where((res_yellow==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_y, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_y:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:
            pos[int(ceny/y)][int(cenx/x)]=2



#----------FOR__white------------
    lv_yellow=np.array([0,0,14])
    uv_yellow=np.array([132,139,255])
    mask_yellow=cv2.inRange(hsv,lv_yellow,uv_yellow)
    res_yellow = cv2.bitwise_and(img2, img2, mask=mask_yellow)
    res_yellow[np.where((res_yellow==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_y, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_y:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        if area > 400:
            pos[int(ceny/y)][int(cenx/x)]=1

#----------FOR__BLUE------------
    lv_blue=np.array([105,146,38])
    uv_blue=np.array([155,255,255])
    mask_blue=cv2.inRange(hsv,lv_blue,uv_blue)
    res_blue = cv2.bitwise_and(img2, img2, mask=mask_blue)
    res_blue[np.where((res_blue==[0,0,0]).all(axis=2))]=[255,255,255]
    ct_b, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in ct_b:
        area = cv2.contourArea(cnt)
        cen = cv2.moments(cnt)
        cenx = int(cen["m10"] / cen["m00"])
        ceny = int(cen["m01"] / cen["m00"])
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        
        
        if area > 400:
                      
            if(len(approx)==3):
                #print("oneway is ")
                #print(nodes[int(ceny/y)][int(cenx/x)])
                #print(approx)

                #pos[int(ceny/y)][int(cenx/x)] = pos[int(ceny/y)][int(cenx/x)]
                approx=ssort(approx)
                #print(approx)
                
                if((approx[0][0][0]+approx[2][0][0])<=2*approx[1][0][0]+10 and (approx[0][0][0]+approx[2][0][0])>=2*approx[1][0][0]-10):
                    if(2*approx[1][0][1]>(approx[0][0][1]+approx[2][0][1])):
                     
                        n=int(ceny/y)*s+int(cenx/x)
                        oneway.append(n)
                        oneway.append(n+s)
                        
                    else:
                        n=int(ceny/y)*s+int(cenx/x)
                        oneway.append(n)
                        oneway.append(n-s)
                elif(approx[0][0][0]<=approx[1][0][0]+5 and approx[0][0][0]>=approx[1][0][0]-5):
                     
                        n=int(ceny/y)*s+int(cenx/x)
                        oneway.append(n)
                        oneway.append(n+1)
                else:

                    n=int(ceny/y)*s+int(cenx/x)
                    oneway.append(n)
                    oneway.append(n-1)
                count=count+1

                

                
            elif(len(approx)==4):
                 #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                 pos[int(ceny/y)][int(cenx/x)]=150+1000
                 hnode[0]=nodes[int(ceny/y)][int(cenx/x)]
                 
            elif(len(approx)>5):
                 #cv2.fillPoly(img, pts=cnt, color=(0,0,0))
                 pos[int(ceny/y)][int(cenx/x)]=200+1000
                 hnode[1]=nodes[int(ceny/y)][int(cenx/x)]
   
   
    pos[s-1][s-1]=0
    print(pos)
    g = Graph(s*s)
    for i in range(s*s):
        for j in range(s*s):
            g.graph[j][i]=0
    for i in range(s*s):
        if(i==0):
            g.graph[1][i]=pos[int(i/s)][i%s]
            g.graph[s][i]=pos[int(i/s)][i%s]
        elif(i==s-1):
            g.graph[s-2][i]=pos[int(i/s)][i%s]
            g.graph[2*s-1][i]=pos[int(i/s)][i%s]
        elif(i==s*s-s):
            g.graph[s*s-2*s][i]=pos[int(i/s)][i%s]
            g.graph[s*s-s+1][i]=pos[int(i/s)][i%s]
        elif(i==s*s-1):
            g.graph[s*s-s-1][i]=pos[int(i/s)][i%s]
            g.graph[s*s-2][i]=pos[int(i/s)][i%s]
        elif(i%s==0):
            g.graph[i-s][i]=pos[int(i/s)][i%s]
            g.graph[i+s][i]=pos[int(i/s)][i%s]
            g.graph[i+1][i]=pos[int(i/s)][i%s]
        elif(i%s==s-1):
            g.graph[i-s][i]=pos[int(i/s)][i%s]
            g.graph[i+s][i]=pos[int(i/s)][i%s]
            g.graph[i-1][i]=pos[int(i/s)][i%s]
        elif(int(i/s)==0):
            g.graph[i-1][i]=pos[int(i/s)][i%s]
            g.graph[i+s][i]=pos[int(i/s)][i%s]
            g.graph[i+1][i]=pos[int(i/s)][i%s]
        elif(int(i/s)==s-1):
            g.graph[i-s][i]=pos[int(i/s)][i%s]
            g.graph[i+1][i]=pos[int(i/s)][i%s]
            g.graph[i-1][i]=pos[int(i/s)][i%s]
        else:
            g.graph[i-s][i]=pos[int(i/s)][i%s]
            g.graph[i+s][i]=pos[int(i/s)][i%s]
            g.graph[i-1][i]=pos[int(i/s)][i%s]
            g.graph[i+1][i]=pos[int(i/s)][i%s]
    print(oneway)
    for j in range(count):
        i=oneway[2*j]
        k=oneway[2*j+1]
        if(i==0):
            g.graph[i][1]=0
            g.graph[i][s]=0
        elif(i==s-1):
            g.graph[i][s-2]=0
            g.graph[i][2*s-1]=0
        elif(i==s*s-s):
            g.graph[i][s*s-2*s]=0
            g.graph[i][s*s-s+1]=0
        elif(i==s*s-1):
            g.graph[i][s*s-s-1]=0
            g.graph[i][s*s-2]=0
        elif(i%s==0):
            g.graph[i][i-s]=0
            g.graph[i][i+s]=0
            g.graph[i][i+1]=0
        elif(i%s==s-1):
            g.graph[i][i-s]=0
            g.graph[i][i+s]=0
            g.graph[i][i-1]=0
        elif(int(i/s)==0):
            g.graph[i][i-1]=0
            g.graph[i][i+s]=0
            g.graph[i][i+1]=0
        elif(int(i/s)==s-1):
            g.graph[i][i-s]=0
            g.graph[i][i+1]=0
            g.graph[i][i-1]=0
        else:
            g.graph[i][i-s]=0
            g.graph[i][i+s]=0
            g.graph[i][i-1]=0
            g.graph[i][i+1]=0
        g.graph[i][k]=pos[int(k/s)][k%s]
        g.graph[k][i]=0
    
    #p.stepSimulation()
    #env.respawn_car()
    im = env.camera_feed()

    img = im[int(r2[1]):int(r2[1]+r2[3]), int(r2[0]):int(r2[0]+r2[2])]
    
    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print(pnode)
    print(hnode)
    print(oneway)
    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    m=0
    k=0
    while True:
        
        im = env.camera_feed()
        
       # print(im.shape)
        c,n,v1,x1,y1=bot_pos(im)
        print(c)
        while v1==-1:

            p.stepSimulation()
            env.move_husky(5,-5,5,-5)
            img1 = env.camera_feed()
            c,n,v1,x1,y1=bot_pos(img1)
        sd,no=g.dijkstra(c)
        if(sd[pnode[1]]<sd[pnode[0]]):
            t=pnode[0]
            pnode[0]=pnode[1]
            pnode[1]=t
        pt=shortest_path(pnode[m],sd,c,s,no)
        print(pt)
        for i in range(8):
            p.stepSimulation()
            env.move_husky(12,12,12,12)
            img1=env.camera_feed()
        move(pt)
        
        print('waiting for destination')

        time.sleep(1)



       
        k=0

        p.stepSimulation()
        env.remove_cover_plate(pnode[m]//s, pnode[m]%s)
        k=read_des()
        print("****")
        print("k is", k)
        sd,no=g.dijkstra(pnode[m])
        pt=shortest_path(hnode[k],sd,pnode[m],s,no)
        print("*--*")
        print(pt)
        
        move(pt)

        print('waiting for destination')
        m=m+1
        print(hnode[k])
        print(pnode[m])
        time.sleep(1)
        
        sd,no=g.dijkstra(hnode[k])
        pt=shortest_path(pnode[m],sd,hnode[k],s,no)
        print("*--*")
        print(pt)
        print("****")
        move(pt)
        print('waiting for destination')
        time.sleep(1)

        k=0
        p.stepSimulation()
        env.remove_cover_plate(pnode[m]//s, pnode[m]%s)
        k=read_des()
        print("****")
        print("k is", k)
        print(k)
        #k=read_des()
        sd,no=g.dijkstra(pnode[m])
        pt=shortest_path(hnode[k],sd,pnode[m],s,no)
        print("*--*")
        print(pt)
        print("****")
        move(pt)
        img1 = env.camera_feed()
        c,n,v1,x1,y1=bot_pos(img1)
        pt=[hnode[k],c]
        print("*--*")
        print(pt)
        print("****")
        move(pt)
        print('task completed')
        print('exiting in 5 sec')
        time.sleep(5)
        break
       
    cv2.destroyAllWindows()    
    p.disconnect()
