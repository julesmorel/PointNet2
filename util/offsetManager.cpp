#include "offsetManager.h"

#include<iostream>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

offsetManager::offsetManager(std::string _filename){
    filename=_filename;
    //initialize offset to 0
    offset_x=0.;
    offset_y=0.;
    offset_stored=false;
    //check in the same folder if the offset file already exist and read it
    boost::filesystem::path localFolder(filename);
    offsetFileName = localFolder.parent_path().string() + boost::filesystem::path::preferred_separator + "offset.txt";
    //if the file exists, then we read it and store the offset    
    if(std::ifstream(offsetFileName)){
        std::ifstream file(offsetFileName);
        if (file.is_open()) {
            std::string line;
            getline(file, line);
            std::vector<std::string> results;
            boost::split(results, line, [](char c){return c == ' ';});
            offset_x=std::stod (results.at(0));
            offset_y=std::stod (results.at(1));
            offset_stored=true;
        }
    }
}

void offsetManager::setOffset(double _offset_x,double _offset_y){
    //if the offset is not already stored
    if(!offset_stored){
        offset_x=_offset_x;
        offset_y=_offset_y;
        offset_stored=true;
        //let's write the file
        std::ofstream outfile;
        outfile.open(offsetFileName, std::ios_base::app);
        outfile<<offset_x<<" "<<offset_y<<std::endl;
        outfile.close();
    }
}