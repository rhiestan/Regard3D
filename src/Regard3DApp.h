#ifndef REGARD3D_APP_H
#define REGARD3D_APP_H

class Regard3DMainFrame;

class Regard3DApp: public wxApp
{
public:
	Regard3DApp();

	virtual bool OnInit();
	virtual int OnExit();
	
private:
	Regard3DMainFrame *pMainFrame_;
};

#endif
