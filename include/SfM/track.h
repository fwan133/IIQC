//
// Created by feng on 7/10/23.
//

#ifndef IIQC_TRACK_H
#define IIQC_TRACK_H

#include <unordered_map>

namespace sfm {

class Track {
public:
    void addElement(long frameId, int kpIdx){
        elements[frameId] = kpIdx;
    };
    int length()const{ return elements.size();}

    const std::unordered_map<long, int> &getElements() const { return elements; }

private:
    std::unordered_map<long, int> elements;      // frameId and keypoint index
};

}


#endif //IIQC_TRACK_H
