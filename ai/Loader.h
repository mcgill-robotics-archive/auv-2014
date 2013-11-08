#ifndef Loader_h
#define Loader_h


class Loader {

 public:

    virtual void Loader();

    virtual something GetMap();

    virtual something GetTasks();

    virtual void newOperation();

    virtual void newOperation();


 private:
    String MapStartOConfigFileName;
	String TasksStartOfConfigFileName;
    File MapFile;
    File TasksFile;
};

#endif // Loader_h
