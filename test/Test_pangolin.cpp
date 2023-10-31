#include "Viewer/viewer.h"

int main(int argc, char **argv){
    Viewer::Ptr viewer = std::shared_ptr<Viewer>(new Viewer());


    return 0;
}