#ifndef Logger_h
#define Logger_h



class Logger {

 public:

    virtual int Logger();

    virtual int WriteString(void  s);

    virtual int WritePose(void  pose);

    virtual int WriteLogConfig(void  Loader);


 private:
    String FileName;
};

#endif // Logger_h
