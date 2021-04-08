#!/usr/bin/env python
# coding: utf-8

# In[26]:


def NormCoords(lista,NORM): # Recibe una lista de coordenadas 
  
    from pyproj.aoi import AreaOfInterest
    from pyproj.database import query_utm_crs_info
    from pyproj import CRS
    from pyproj import Proj
    import math 
    import numpy as np
    import pandas as pd
    
    Lat = np.array(lista)[:,0]
    Long = np.array(lista)[:,1]
    
    utm_crs_list = query_utm_crs_info(
        datum_name="WGS 84",
        area_of_interest=AreaOfInterest(
            west_lon_degree=Lat.mean(),
            south_lat_degree=Long.mean(),
            east_lon_degree=Lat.mean(),
            north_lat_degree=Long.mean(),
        ),
    )

    utm_crs = CRS.from_epsg(utm_crs_list[0].code)
    
    myProj = Proj(utm_crs_list[0].name)
    
    UTMx,UTMy = myProj(Long, Lat, inverse=False)

    MxLon = UTMx.max()
    MnLon = UTMx.min()
    MxLat = UTMy.max()
    MnLat = UTMy.min()
    
    bx = (UTMx.max()-UTMx.min())
    by = (UTMy.max()-UTMy.min())
    
    NormLon = []
    NormLat = []
    
    for i in range(0,len(UTMx)):
        
        NormLon.append((UTMx[i]-MxLon)*NORM/bx + NORM)
        NormLat.append((UTMy[i]-MxLat)*NORM/by + NORM)
        
    Info = []
    
    for i in range(0,len(NormLon)):
        Info.append([NormLon[i],NormLat[i]])
        
    return(Info)


# In[27]:


def SquareCoord(lista,NORM): # Recibe una lista de coordenadas 
  
    import math
    import numpy as np 
    
    a = 1/111111 
    square = [[0,0]]
    
    for i in range(1,len(lista)):
        
        prevposition = lista[i-1]
        
        position = lista[i]
        cose = math.cos(math.radians(position[0])) #position[0] es LATITUD!
        
        incrLat = position[0] - prevposition[0]
        incrY = incrLat/a
        
        incrLong = position[0] - prevposition[0]
        incrX = incrLong*cose/a
        
        square.append(list(np.array(square[i-1]) + np.array([incrY,incrX]))) # LAT LONG tiene que ser el formato!!
     
    Lat = np.array(lista)[:,0]
    Lon = np.array(lista)[:,1]

    mxlat = max(Lat)
    mxlon = max(Lon)

    mnlat = min(Lat)
    mnlon = min(Lon)
    
    
    bx = (mxlon-mnlon)
    by = (mxlat-mnlat)
    
    NormLon = []
    NormLat = []
    
    for i in range(0,len(square)):
        
        NormLon.append((Lon[i]-mxlon)*NORM/bx + NORM)
        NormLat.append((Lat[i]-mxlat)*NORM/by + NORM)
        
        
    Info = []
    
    for i in range(0,len(NormLon)):
        Info.append([NormLon[i],NormLat[i]])
        

    return(Info)


# In[28]:


#Lista = [[-41.01, -3.86], [-41.05,-3.88], [-42.05, -3.87], [-41.07, -3.89]]


# In[31]:


#NormCoords(Lista,1) # Devuelve un polígono que NO es lineal (utiliza referencias elípticas)


# In[32]:


#SquareCoord(Lista,1) # Devuelve un polígono lineal. El segundo argumento es la escala. 0-X


# In[ ]:


# NECESITA LAT LON. En ese orden.

