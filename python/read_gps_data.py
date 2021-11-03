#coding:utf-8
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

# 'A program used to get GPS information in picture'

# __author__ = 'Albert Yang'

import exifread
import re
import os, os.path as osp

def FindGPSTime(filePath):
    GPS={}
    Data=""
    f=open(filePath,'rb')
    tags=exifread.process_file(f)

    # print(tags)
    # print("f:",f.read())
    # print("tags:",tags)
    # for key in tags:
    #    print(key)

    for tag,value in tags.items():
        if re.match('GPS GPSLatitudeRef',tag):
            GPS['GPSLatitudeRef(纬度标识)']=str(value)
        elif re.match('GPS GPSLongitudeRef',tag):
            GPS['GPSLongitudeRef(经度标识)']=str(value)
        elif re.match('GPS GPSAltitudeRef',tag):
            GPS['GPSAltitudeRef(高度标识)']=str(value)
        elif re.match('GPS GPSLatitude',tag):
            try:
                match_result=re.match('\[(\w*), (\w*), (\w.*)/(\w.*)\]',str(value)).groups()   #匹配临近的字符
                # GPS['GPSLatitude(纬度)']=int(match_result[0]),int(match_result[1]),int(match_result[2])/int(match_result[3])
                GPS['GPSLatitude(纬度)']=float(match_result[0]) +  float(match_result[1]) / float(60.0) + float(match_result[2])/float(match_result[3]) / float(60.0) / float(60.0)
            except:
                GPS['GPSLatitude(纬度)']=str(value)
        elif re.match('GPS GPSLongitude',tag):
            try:
                match_result=re.match('\[(\w*), (\w*), (\w.*)/(\w.*)\]',str(value)).groups()
                GPS['GPSLongitude(经度)']=float(match_result[0]) +  float(match_result[1]) / float(60.0) + float(match_result[2])/float(match_result[3]) / float(60.0) / float(60.0)
            except:
                GPS['GPSLongitude(经度)']=str(value)
        elif re.match('GPS GPSAltitude',tag):
            # match_result=re.match('\[(\w.*)/(\w.*)\]',str(value))
            match_result=re.match('(-?\d+)\/(\d+)',str(value)).groups()
            GPS['GPSAltitude(高度)'] = float(match_result[0]) / float(match_result[1])
        elif re.match('Image DateTime',tag):
            Data=str(value)
    
    return {'GPS 信息':GPS,'时间信息':Data}
    #http: // www.gpsspg.com / maps.htm

if __name__=='__main__':
    images_dir = "/media/zmc/0053-2C75/红外以及可见光照片/kejianguang/"
    save_path = osp.join(images_dir, "gps_pose.txt")
    images_path = os.listdir(images_dir)
    images_path = sorted(images_path)
    with open(save_path,"w") as f:
        for image_path in images_path:
            image_path_all = osp.join(images_dir, image_path)
            print(image_path)
            # print(FindGPSTime(image_path_all))
            gps_inf = FindGPSTime(image_path_all)
            if len(gps_inf["GPS 信息"]) == 0:
                continue
                print("empty")
            input_line = str(image_path + " " + str(gps_inf["GPS 信息"]["GPSLongitude(经度)"]) + " " + str(gps_inf["GPS 信息"]["GPSLatitude(纬度)"]) +" " + str(gps_inf["GPS 信息"]["GPSAltitude(高度)"]) + "\n")
            f.write(input_line)


