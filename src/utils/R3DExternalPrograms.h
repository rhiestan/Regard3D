/**
 * Copyright (C) 2015 Roman Hiestand
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef R3DEXTERNALPROGRAMS_H
#define R3DEXTERNALPROGRAMS_H

class R3DExternalPrograms
{
public:

	bool initialize();

	const wxString &getCMVSPath() { return cmvsPath_; }
	const wxString &getPMVSPath() { return pmvsPath_; }
	const wxString &getGenOptionPath() { return genOptionPath_; }
	const wxString &getPoissonReconPath() { return poissonReconPath_; }
	const wxString &getSurfaceTrimmerPath() { return surfaceTrimmerPath_; }
	const wxString &getMakescenePath() { return makescenePath_; }
	const wxString &getTexReconPath() { return texreconPath_; }
	const wxString &getDMReconPath() { return dmreconPath_; }
	const wxString &getScene2PsetPath() { return scene2psetPath_; }
	const wxString &getFSSReconPath() { return fssreconPath_; }
	const wxString &getMeshCleanPath() { return meshcleanPath_; }
	const wxString &getCMPMVSPath() { return cmpmvsPath_; }
	const wxString &getSMVSReconPath() { return smvsreconPath_; }
	const wxString &getSMVSReconSSE41Path() { return smvsreconSSE41Path; }

	const wxArrayString &getAllPaths() { return allPaths_; }

	static R3DExternalPrograms &getInstance() { return instance_; }

private:
	R3DExternalPrograms();
	virtual ~R3DExternalPrograms();
	bool checkExecutable(const wxString &path, const wxString &name, const wxString &extension,
		wxString &outFullPath);

	bool initialized_;
	wxString cmvsPath_, pmvsPath_, genOptionPath_;
	wxString poissonReconPath_, surfaceTrimmerPath_;
	wxString makescenePath_, texreconPath_;
	wxString dmreconPath_, scene2psetPath_, fssreconPath_, meshcleanPath_;
	wxString cmpmvsPath_;
	wxString smvsreconPath_, smvsreconSSE41Path;
	wxArrayString allPaths_;

	static R3DExternalPrograms instance_;
};

#endif
