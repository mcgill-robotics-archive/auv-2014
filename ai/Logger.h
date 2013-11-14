#ifndef Logger_h
#define Logger_h



class Logger {

 public:

    Logger();

    virtual int WriteString(std::string  s);

    virtual int WritePose(PoseObj::PoseObj  pose);

//    virtual int WriteLogConfig(void  Loader);


 private:
    std::string FileName;
};

#endif // Logger_h
