#ifndef IIQC_CONFIG_H
#define IIQC_CONFIG_H

#include <iostream>
#include <memory>

#include <opencv4/opencv2/core/core.hpp>

class Config {
public:
    Config(const Config &) = delete;

    ~Config(){
        if (file_->isOpened())
            file_->release();
        delete file_;
    }

    Config &operator=(const Config &) = delete;

    static std::shared_ptr<Config> instance(){
        static std::shared_ptr<Config> config_ = nullptr;
        if (config_ == nullptr)
            config_ = std::shared_ptr<Config>(new Config);

        return config_;
    };

    void setParameterFile(const std::string &filename){
        file_ = new cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (!file_->isOpened()) {
            std::cerr << "parameter file " << filename << " doesn't exist.\n";
            file_->release();
        }
    };

    template<typename T>
    T get(const std::string &key) const {
        return T(file_->operator[](key));
    }

private:
    Config() {}

    cv::FileStorage *file_;
};

#endif //IIQC_CONFIG_H
