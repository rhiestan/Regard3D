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
#include "CommonIncludes.h"
#include "SharedGLContext.h"

#include <wx/utils.h>

// Uncomment if GLEW is used
//#define WE_HAVE_GLEW

// In debug mode on Windows, try to create a OpenGL debug context
#if defined(_DEBUG) && defined(__WXMSW__)
//#define CREATE_DEBUG_CONTEXT
#endif

#if !defined(WE_HAVE_GLEW)
#define WGL_CONTEXT_DEBUG_BIT_ARB 0x0001
#define WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB 0x0002
#define WGL_CONTEXT_FLAGS_ARB 0x2094
#endif

SharedGLContext *SharedGLContext::pInstance_ = NULL;

// Copied from http://www.opengl.org/archives/resources/features/OGLextensions/
int isExtensionSupported(const char *extension)
{
	const GLubyte *extensions = NULL;
	const GLubyte *start;
	GLubyte *where, *terminator;

	/* Extension names should not have spaces. */
	where = (GLubyte *) strchr(extension, ' ');
	if (where || *extension == '\0')
		return 0;
	extensions = glGetString(GL_EXTENSIONS);
	/* It takes a bit of care to be fool-proof about parsing the
	 OpenGL extensions string. Don't be fooled by sub-strings,
	 etc. */
	start = extensions;
	for (;;)
	{
		where = (GLubyte *) strstr((const char *) start, extension);
		if (!where)
			break;
		terminator = where + strlen(extension);
		if (where == start || *(where - 1) == ' ')
			if (*terminator == ' ' || *terminator == '\0')
				return 1;
		start = terminator;
	}
	return 0;
}

SharedGLContext::SharedGLContext(wxGLCanvas *win)
	: wxGLContext(win)
{
	assert(pInstance_ == NULL);
	pInstance_ = this;

#ifdef CREATE_DEBUG_CONTEXT
	wglMakeCurrent(win->GetHDC(), m_glContext);

	// Check for GL_ARB_debug_output
	if(isExtensionSupported("GL_ARB_debug_output") == 0)
		return;

	// Open debug context if available

#	if defined(WE_HAVE_GLEW)
	glewInit();
	if(WGLEW_ARB_create_context)
#	else
	typedef HGLRC (WINAPI * PFNWGLCREATECONTEXTATTRIBSARBPROC) (HDC hDC, HGLRC hShareContext, const int* attribList);
	PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB;
	wglCreateContextAttribsARB = (PFNWGLCREATECONTEXTATTRIBSARBPROC)wglGetProcAddress("wglCreateContextAttribsARB");
	if(wglCreateContextAttribsARB != NULL)
#	endif
	{
		int attribs[] =
		{
			WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_DEBUG_BIT_ARB | WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
			0
		};
		HGLRC newContext = wglCreateContextAttribsARB(win->GetHDC(), NULL, attribs);
		wglMakeCurrent(NULL, NULL); 
		wglDeleteContext(m_glContext);
		m_glContext = newContext;
	}

	wglMakeCurrent(win->GetHDC(), m_glContext);
#	if defined(WE_HAVE_GLEW)
	glewInit();
	if(GLEW_ARB_debug_output)
#	else
	typedef void (APIENTRY *GLDEBUGPROCARB)(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const char* message, GLvoid* userParam);
	typedef void (WINAPI * PFNGLDEBUGMESSAGECALLBACKARBPROC) (GLDEBUGPROCARB callback, void* userParam);
	typedef void (WINAPI * PFNGLDEBUGMESSAGECONTROLARBPROC) (GLenum source, GLenum type, GLenum severity, GLsizei count, const GLuint* ids, GLboolean enabled);
	PFNGLDEBUGMESSAGECALLBACKARBPROC glDebugMessageCallbackARB;
	PFNGLDEBUGMESSAGECONTROLARBPROC glDebugMessageControlARB;
	glDebugMessageCallbackARB = (PFNGLDEBUGMESSAGECALLBACKARBPROC)wglGetProcAddress("glDebugMessageCallbackARB");
	glDebugMessageControlARB = (PFNGLDEBUGMESSAGECONTROLARBPROC)wglGetProcAddress("glDebugMessageControlARB");
	if(glDebugMessageCallbackARB != NULL
		&& glDebugMessageControlARB != NULL)
#	endif
	{
		glDebugMessageCallbackARB(&debugCallback, NULL);
		glDebugMessageControlARB(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
	}
	wglMakeCurrent(NULL, NULL);

/*	Code for glX/X11 crashes
	typedef GLXContext ( * PFNGLXCREATECONTEXTATTRIBSARBPROC) (Display* dpy, GLXFBConfig config, GLXContext share_context, Bool direct, const int *attrib_list);
	PFNGLXCREATECONTEXTATTRIBSARBPROC glXCreateContextAttribsARB;
	glXCreateContextAttribsARB = (PFNGLXCREATECONTEXTATTRIBSARBPROC)glXGetProcAddress((const GLubyte*)"glXCreateContextAttribsARB");
	if(glXCreateContextAttribsARB != NULL)
	{
		int attribs[] =
		{
			GLX_CONTEXT_FLAGS_ARB, GLX_CONTEXT_DEBUG_BIT_ARB | GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
			0
		};

	        GLXFBConfig *fbc = win->GetGLXFBConfig();
	        wxCHECK_RET( fbc, wxT("invalid GLXFBConfig for OpenGL") );

		GLXContext newContext = glXCreateContextAttribsARB(wxGetX11Display(), fbc[0], NULL, GL_TRUE, attribs);
		setNoActiveContext(win);
		//glXDestroyContext( wxGetX11Display(), m_glContext );	// No access to private data member
		m_glContext2 = newContext;
	}
*/
#endif
}

SharedGLContext::~SharedGLContext()
{
	pInstance_ = NULL;
}

#if !defined(WE_HAVE_GLEW)
#define GL_DEBUG_SOURCE_API_ARB 0x8246
#define GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB 0x8247
#define GL_DEBUG_SOURCE_SHADER_COMPILER_ARB 0x8248
#define GL_DEBUG_SOURCE_THIRD_PARTY_ARB 0x8249
#define GL_DEBUG_SOURCE_APPLICATION_ARB 0x824A
#define GL_DEBUG_SOURCE_OTHER_ARB 0x824B
#define GL_DEBUG_TYPE_ERROR_ARB 0x824C
#define GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB 0x824D
#define GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB 0x824E
#define GL_DEBUG_TYPE_PORTABILITY_ARB 0x824F
#define GL_DEBUG_TYPE_PERFORMANCE_ARB 0x8250
#define GL_DEBUG_TYPE_OTHER_ARB 0x8251
#define GL_DEBUG_SEVERITY_HIGH_ARB 0x9146
#define GL_DEBUG_SEVERITY_MEDIUM_ARB 0x9147
#define GL_DEBUG_SEVERITY_LOW_ARB 0x9148
#endif

void CALLBACK SharedGLContext::debugCallback(unsigned int source, unsigned int type,
	unsigned int id, unsigned int severity,
	int length, const char* message, void* userParam)
{
	char buf[10240];
	const char *sourceStr = "Unknown";
	if(source == GL_DEBUG_SOURCE_API_ARB)
		sourceStr = "API";
	else if(source == GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB)
		sourceStr = "Window system";
	else if(source == GL_DEBUG_SOURCE_SHADER_COMPILER_ARB)
		sourceStr = "Shader compiler";
	else if(source == GL_DEBUG_SOURCE_THIRD_PARTY_ARB)
		sourceStr = "Third party";
	else if(source == GL_DEBUG_SOURCE_APPLICATION_ARB)
		sourceStr = "Application";
	const char *typeStr = "Unknown";
	if(type == GL_DEBUG_TYPE_ERROR_ARB)
		typeStr = "Error";
	else if(type == GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB)
		typeStr = "Deprecated";
	else if(type == GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB)
		typeStr = "Undefined";
	else if(type == GL_DEBUG_TYPE_PORTABILITY_ARB)
		typeStr = "Portability";
	else if(type == GL_DEBUG_TYPE_PERFORMANCE_ARB)
		typeStr = "Performance";
	const char *severityStr = "Low";
	if(severity == GL_DEBUG_SEVERITY_HIGH_ARB)
		severityStr = "High";
	else if(severity == GL_DEBUG_SEVERITY_MEDIUM_ARB)
		severityStr = "Medium";

	sprintf(buf, "OpenGL error callback, source: %s, type: %s, id: %d, severity: %s, message: %s\n",
		sourceStr, typeStr, id, severityStr, message);
#if defined(__WXMSW__)
	OutputDebugStringA(buf);
#elif defined(__WXX11__) || defined(__WXGTK__)
	printf("%s", buf);
#endif
}

#if defined(__WXMAC__)
#	include "CGLCurrent.h"
#endif

/**
 * This method clears the active OpenGL context.
 *
 * Unfortunately this is not possible using wxWidgets methods.
 */
void SharedGLContext::setNoActiveContext(wxGLCanvas *pGLCanvas)
{
#if defined(__WXMSW__)
	wglMakeCurrent((HDC) pGLCanvas->GetHDC(), NULL);
#elif defined(__WXX11__) || defined(__WXGTK__)
	// Unfortunately wxGLContext::MakeCurrent is private, so we have to copy the code here...
	if (wxGLCanvas::GetGLXVersion() >= 13)
#	if wxCHECK_VERSION(2, 9, 0)
		glXMakeContextCurrent( wxGetX11Display(), None, None, NULL);
	else // GLX <= 1.2 doesn't have glXMakeContextCurrent()
		glXMakeCurrent( wxGetX11Display(), None, NULL);
#	else
		glXMakeContextCurrent( (Display *)wxGetDisplay(), None, None, NULL);
	else // GLX <= 1.2 doesn't have glXMakeContextCurrent()
		glXMakeCurrent( (Display *)wxGetDisplay(), None, NULL);
#	endif
#elif defined(__WXMAC__)
	CGLSetCurrentContext(NULL);
#else
#	error "Unsupported platform: Please add platform-specific code here"
#endif
}

