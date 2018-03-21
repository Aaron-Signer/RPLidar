//
//  line.h
//  Lines
//
//  Created by Ben Shaffer on 3/14/15.
//  Copyright (c) 2015 Ben Shaffer. All rights reserved.
//

#ifndef Lines_line_h
#define Lines_line_h

#include "point.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>



struct line{
    std::vector<point> points;
};

bool comparePoints(point p1, point p2){
    return (getX(p1)<getX(p2));
}

const double lineTolerance=30;
const std::size_t minPointsPerLine=0;
const std::size_t maxMissed=3;

line makeLine(point startPoint, point endPoint){
    line l;
    l.points.push_back(startPoint);
    l.points.push_back(endPoint);
    return l;
}

double distanceToPoint(line l, point p){
    double deltaY= getY(l.points[1])-getY(l.points[0]);
    double deltaX= getX(l.points[1])-getX(l.points[0]);
    double lineDistance= sqrt(deltaY*deltaY+deltaX*deltaX);
    return abs(deltaY*getX(p)-deltaX*getY(p)+getX(l.points[1])*getY(l.points[0])-getX(l.points[0])*getY(l.points[1]))/lineDistance;
}

bool accumulate(line& l, point p){
    if (distanceToPoint(l,p)<lineTolerance){
        l.points.push_back(p);
        return true;
    }
    else{
        return false;
    }
}

line findLine(std::vector<point> points){
    line l;
    l=makeLine(points[0],points[1]);
    std::size_t missed=0;
    for(std::size_t i=2; i< points.size() && missed < maxMissed;i++){
           if(!accumulate(l, points[i])){
                missed++;
           }
           else{
                missed=0;
           }
    }
    return l;
}

void sortLine(line l){
    std::sort(l.points.begin(), l.points.end(), comparePoints);
}

std::vector<line> findLines(std::vector<point> allPoints){
    std::vector<line> allLines;
    
    while(allPoints.size()>10){
        line l;
        l=findLine(allPoints);
        if(l.points.size()>minPointsPerLine){
            sortLine(l);
            allLines.push_back(l);
            for(std::size_t i=0; i<l.points.size(); i++){
                std::vector<point>::iterator it=find(allPoints.begin(), allPoints.end(),   l.points[i]);
                allPoints.erase(it);
             }
        }
        else{
            sortLine(l);
            point p1, p2;
            p1=l.points[0];
            p2=l.points[1];
            std::vector<point>::iterator it=find(allPoints.begin(), allPoints.end(),   l.points[0]);
            allPoints.erase(it);
            std::vector<point>::iterator it2=find(allPoints.begin(), allPoints.end(),   l.points[1]);
            allPoints.erase(it2);
           // allPoints.push_back(p1);
           // allPoints.push_back(p2);
        }
    }
    return allLines;
}




void endPoints(line l, point& p1, point& p2){
    p1=l.points[0];
    p2=l.points[l.points.size()-1];
}

void writeWalls (std::string fileName, std::vector<line> input){
    std::ofstream fout(fileName.c_str());
    for(std::size_t i=0; i<input.size(); i++){
        point p1;
        point p2;
        endPoints(input[i], p1, p2);
        fout << getX(p1) << " " << getY(p1) << std::endl;
        fout << getX(p2) << " " << getY(p2) << std::endl << std::endl;
    }
    fout.close();
}

double distancePointToSegment(line l, point p){
    point differance;
    point v=l.points[0];
    point w=l.points[l.points.size()-1];
    differance=subtractPoints(v, w);
    double l2 = lengthSquared(differance);
    if(getX(differance)==0 && getY(differance)==0){
        return distanceBetweenPoints(p, w);
    }
    double dot=dotProduct(subtractPoints(p, v), subtractPoints(w, v))/l2;
    if (dot>1){
        return distanceBetweenPoints(p,w);
    }
    else if(dot<0){
        return distanceBetweenPoints(p,v);
    }
    else{
        return distanceToPoint(l,p);
    }
}

double slope(line l){
    point first = l.points[0];
    point last = l.points[(l.points.size()-1)];
    if((getX(last)-getX(first))==0){
        if (getY(last)-getY(first)>0){
            return 1000;
        }
        else{
            return -1000;
        }
    }
    else{
        return ((getY(last)-getY(first))/(getX(last)-getX(first)));
    }
}

std::vector<line> mergeLines(std::vector<line> lineList){
    std::vector<line> result;
   for (std::size_t i=0; i<lineList.size(); i++){
        double lineSlope= slope(lineList[i]);
        bool merged=false;
        for(std::size_t j=0; j<result.size(); j++){
            double otherSlope= slope(result[j]);
            double d1= distanceBetweenPoints(lineList[i].points[0], result[j].points.back());
            double d2= distanceBetweenPoints(lineList[i].points[0], result[j].points[0]);
            double d3= distanceBetweenPoints(lineList[i].points.back(), result[j].points.back());
            double d4= distanceBetweenPoints(lineList[i].points.back(), result[j].points[0]);
            if (abs(lineSlope-otherSlope)<25 && (d1<20||d2<20||d3<20||d4<20)){
                result[j].points.insert(result[j].points.begin(), lineList[i].points.begin(), lineList[i].points.end());
                merged=true;
            }
        }
        if(!merged){
            result.push_back(lineList[i]);
        }
   }
   for_each(result.begin(), result.end(), sortLine);
   return result;
}


#endif
