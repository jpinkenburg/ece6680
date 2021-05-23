import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib import cm
from matplotlib import colors as cl
import colorsys
from matplotlib.collections import LineCollection

def make_plot(pos,startTime,endTime,startColor=np.array([1,0,0,1]),endColor=np.array([0,0,1,1])):
    fig,ax = plt.subplots(1,1)
    N = len(pos)
    colors = np.zeros([N,4])
    for i in range(4):
        colors[:,i] = np.linspace(startColor[i],endColor[i],N)
    cmap = ListedColormap(colors)
    for i in range(len(pos)-1):
      ax.plot([pos[i][0],pos[i+1][0]],[pos[i][1],pos[i+1][1]],color=colors[i,:3],linewidth=2)
    ax.set(xlabel='Body X Position', ylabel='Body Y Position')
    cbar = plt.colorbar(cm.ScalarMappable(norm=cl.Normalize(vmin=0,vmax=20),cmap=cmap))
    cbar.ax.get_yaxis().labelpad = 15
    cbar.ax.set_ylabel('time (s)', rotation=270)
    #fig.colorbar(cm.ScalarMappable(norm=cl.Normalize(vmin=0,vmax=20),cmap=cmap))
    return fig,ax

def make_graded_limb_plot(limbPos,limbVels,startTime,endTime):
    #plt.style.use('dark_background')
    fig,(ax1,ax2) = plt.subplots(2) #for position and velocity plots
    N = len(limbPos)
    c = np.ones([4,N,4])
    startColors = np.zeros([4,4])
    startColors[0,:] = [0,0,1,1] #blue
    startColors[1,:] = [12/255,100/255,0,1] #green
    startColors[2,:] = [1,0,0,1] #red
    startColors[3,:] = [1,157/255,0,1] #orange
    hues = np.zeros(4)
    for i in range(4): #convert to HSV, all S values should be 1
        hues[i] = np.array(colorsys.rgb_to_hsv(startColors[i,0],startColors[i,1],startColors[i,2]))[0]
    
    #now space saturations from 1 to 0.4
    sList = np.linspace(1,0.4,N)

    cmaps = []
    for i in range(4):
        for j in range(N):
            c[i,j,:3] = np.array(colorsys.hsv_to_rgb(hues[i],min(1.6-sList[j],1),sList[j]))
        cmaps.append(ListedColormap(c[i,:,:]))
    #turn into colormaps

    cmaps[0] = make_colormap([1,0,0],[1,1,0],N)
    cmaps[1] = make_colormap([1,0,1],[0,0,1],N)
    cmaps[2] = make_colormap([0,1,0],[1,0,0],N)
    cmaps[3] = make_colormap([0,0,1],[0,1,0],N)
    for i in range(3):
        cmaps[i+1] = cmaps[0]
    #now plot the lines
    print(c.shape)
    print(limbPos.shape)
    print(limbVels.shape)
    minx = []
    miny = []
    maxx = []
    maxy = []
    for j in range(4): 
        x = limbPos[:,0,j]
        y = limbPos[:,1,j]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        norm = plt.Normalize(0, 20)
        lc = LineCollection(segments, cmap=cmaps[j], norm=norm)
        print(segments.shape)
        lc.set_array(np.linspace(0,20,N-1))
        lc.set_linewidth(2)
        line = ax1.add_collection(lc)
        #fig.colorbar(line, ax=axs[0])
        minx.append(x.min())
        miny.append(y.min())
        maxx.append(x.max())
        maxy.append(y.max())
    ax1.set_xlim(min(minx)-0.1*abs(min(minx)-max(maxx)), max(maxx)+0.1*abs(max(maxx)))
    ax1.set_ylim(min(miny)-0.1*abs(min(miny)),max(maxy)+0.1*abs(max(maxy)))

        #for i in range(len(limbPos)-1):
        #    ax1.plot([limbPos[i][0][j],limbPos[i+1][0][j]],[limbPos[i][1][j],limbPos[i+1][1][j]],color=cmaps[j],linewidth=3)
        #    #ax1.scatter(limbPos[i][0][j],limbPos[i][1][j],color=c[j,i,:3],s=0.5)
        #    ax2.plot([limbVels[i][j],limbVels[i+1][j]],[limbVels[i][j],limbVels[i+1][j]],color=c[j,i,:3],linewidth=3)
        
    return fig,ax1,ax2
    
def make_colormap(color1,color2,N):
    twocolor = np.ones([N,4])
    for i in range(3):
        twocolor[:,i] = np.linspace(color1[i],color2[i],N)
    return ListedColormap(twocolor)