# dispatch/DispatchTools.py
import yaml

def convert_coordinates_map_to_img(realx, realy, resolution, originx, originy):
    pixelx = (realx - originx) / resolution
    pixely = (realy - originy) / resolution
    return (pixelx, pixely)

def convert_coordinates_image_to_map(pixelx, pixely, resolution, originx, originy):
    realx = pixelx * resolution + originx
    realy = pixely * resolution + originy
    return (realx, realy)
