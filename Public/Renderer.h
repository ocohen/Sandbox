#ifndef TINY_RENDERER_H
#define TINY_RENDERER_H

#if _WIN32 || _WIN64
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "Vector3.h"
#include "Transform.h"

//TODO: move these into a proper static
const Vector3 Red = { 1.f, 0.f, 0.f };
const Vector3 Green = Vector3(0.f, 1.f, 0.f);
const Vector3 Blue = Vector3(0.f, 0.f, 1.f);

class Renderer
{
public:
    Renderer(int inWidth, int inHeight);
    void setBackground(const Vector3& color);
    void initGL();
    void clear();
    void drawPoint(const Vector3& pt, const Vector3* color = 0, float thickness = 3.f);
    void drawLine(const Vector3& start, const Vector3&end, const Vector3* color = 0, float thickness = 1.f);
    void drawTriangle(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3* color = 0, float thickness = 1.f);
    void drawSolidTriangle(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3* color);
    void drawOrientedCircles(const Transform& tm, float radius, int numSections=16, float thickness=1.f);
    void drawCross(const Transform& tm, float size, float thickness = 1.f);
    //tris is just an index buffer, but we treat it as triangles (num indices = numTris * 3)
    void drawMesh(const Vector3* vertices, const unsigned short* tris, int numTris, const Vector3* color = 0, bool solid=true, float thickness =1.f);
    void drawSphere(const Vector3& center, float radius, int numSections=16, const Vector3* color = 0, float thickness=1.f);
    void setCameraPosition(const Vector3& eye);
    void setCameraLookAt(const Vector3& eye, const Vector3& target, const Vector3& up);
    void setCameraLense(float angleFOV, float nearPlane=1.f, float farPlane=1000.f);
    void setDimensions(int width, int height);
    void flush();
private:
    float width;
    float height;
    float mv[16];
    float projection[16];

    void drawVertex(const Vector3& v);
};

inline void Renderer::drawVertex(const Vector3& v)
{
    glVertex3f(v[0], v[1], v[2]);
}

inline Renderer::Renderer(int inWidth, int inHeight)
{
    initGL();
    setDimensions(inWidth, inHeight);
    
    Vector3 eye(0.f, 0.f, 0.f);
    Vector3 target(0.f, 0.f, -1.f);
    Vector3 up(0.f, 0.f, 1.f);
    setCameraLookAt(eye,target,up);
    setCameraLense(90);
}

inline void Renderer::setBackground(const Vector3& color)
{
    glClearColor(color[0], color[1], color[2], 0.f);
}

inline void Renderer::initGL()
{
    glShadeModel( GL_SMOOTH );
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
    glClearDepth( 1.0f );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );
}

inline void Renderer::setDimensions(int inWidth, int inHeight)
{
    width = (float)inWidth;
    height = (float)inHeight;
    
    glViewport(0, 0, inWidth, inHeight);
}

inline void Renderer::clear()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

inline void setColor(const Vector3* color)
{
    if(color)
    {
        glColor3f(color->x, color->y, color->z);
    }else
    {
        glColor3f(1.f, 1.f, 1.f);
    }
}

inline void Renderer::drawPoint(const Vector3& pt, const Vector3* color, float thickness)
{
    setColor(color);   
    glPointSize(thickness);
    glBegin(GL_POINTS);
    drawVertex(pt);
    glEnd();
    
}

inline void Renderer::drawLine(const Vector3& start, const Vector3& end, const Vector3* color, float thickness)
{
    glLineWidth(thickness);
    setColor(color);
    glBegin(GL_LINES);
    drawVertex(start);
    drawVertex(end);
    glEnd();
}

inline void Renderer::drawTriangle(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3* color, float thickness)
{
    drawLine(a,b,color,thickness);
    drawLine(b,c,color,thickness);
    drawLine(c,a,color,thickness);
}

inline void Renderer::drawSolidTriangle(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3* color)
{
    setColor(color);
    glBegin(GL_TRIANGLES);
    drawVertex(a);
    drawVertex(b);
    drawVertex(c);
    glEnd();
}

inline void Renderer::drawOrientedCircles(const Transform& tm, float radius, int numSections/* =16 */, float thickness/* =1.f */)
{
    const Vector3 localX = tm.transformVector(Vector3(1.f, 0.f, 0.f));
    const Vector3 localY = tm.transformVector(Vector3(0.f, 1.f, 0.f));
    const Vector3 localZ = tm.transformVector(Vector3(0.f, 0.f, 1.f));

    const float rads = 2.f*PI / numSections;
    
    for(int i=0; i < numSections; ++i)
    {
        const float curRad = rads*i;
        const float nextRad = rads*(i+1);
        const Vector3 a = tm.translation + radius * (cos(curRad) * localX + sin(curRad) * localY);
        const Vector3 b = tm.translation + radius * (cos(nextRad) * localX + sin(nextRad) * localY);

        drawLine(a, b, &Blue, thickness);
    }

    for (int i = 0; i < numSections; ++i)
    {
        const float curRad = rads*i;
        const float nextRad = rads*(i + 1);
        const Vector3 a = tm.translation + radius * (cos(curRad) * localZ + sin(curRad) * localY);
        const Vector3 b = tm.translation + radius * (cos(nextRad) * localZ + sin(nextRad) * localY);

        drawLine(a, b, &Red, thickness);
    }

    for (int i = 0; i < numSections; ++i)
    {
        const float curRad = rads*i;
        const float nextRad = rads*(i + 1);
        const Vector3 a = tm.translation + radius * (cos(curRad) * localZ + sin(curRad) * localX);
        const Vector3 b = tm.translation + radius * (cos(nextRad) * localZ + sin(nextRad) * localX);

        drawLine(a, b, &Green, thickness);
    }
}

inline void Renderer::drawCross(const Transform& tm, float size, float thickness /* = 1.f */)
{
    const Vector3 localX = tm.transformVector(Vector3(1.f, 0.f, 0.f));
    const Vector3 localY = tm.transformVector(Vector3(0.f, 1.f, 0.f));
    const Vector3 localZ = tm.transformVector(Vector3(0.f, 0.f, 1.f));

    drawLine(tm.translation, tm.translation + localX * size , &Red, thickness);
    drawLine(tm.translation, tm.translation + localY * size , &Green, thickness);
    drawLine(tm.translation, tm.translation + localZ * size , &Blue, thickness);
}

inline void Renderer::drawMesh(const Vector3* vertices, const unsigned short* tris, int numTris, const Vector3* color, bool solid, float thickness)
{
    setColor(color);
    if(solid)
    {
        glVertexPointer(3, GL_FLOAT, sizeof(vertices[0]), &vertices[0].x);    
        glDrawElements(GL_TRIANGLES, numTris*3, GL_UNSIGNED_SHORT, tris);
    }else
    {
        for(int i=0; i<numTris; i++)
        {
            int a = tris[i*3];
            const Vector3& vA = vertices[a];
        
            int b = tris[i*3+1];
            const Vector3& vB = vertices[b];
        
            int c = tris[i*3+2];
            const Vector3& vC = vertices[c];
        
            if(solid)
            {
                drawSolidTriangle(vA,vB,vC,color);            
            }else
            {
                drawTriangle(vA,vB,vC,color,thickness);
            }

        } 
    }
}

inline void Renderer::drawSphere(const Vector3& center, float radius, int numSections, const Vector3* color, float thickness)
{
    setColor(color);
    Vector3 a;
    Vector3 b;
    
    float rads = 2.f*PI/numSections;
    float unit = 1.f/numSections;
    
    for(int i=0; i<numSections; i++)
    {
        float offset = radius*unit*i*2 - radius;
        float r = sqrt(radius*radius - offset*offset);
        for(int j=0; j<numSections; j++)
        {
            a[0] = center[0] + r * cos(rads*j);
            a[1] = center[1] + offset;
            a[2] = center[2] + r * sin(rads*j);
        
            b[0] = center[0] + r * cos(rads*(j+1));
            b[1] = center[1] + offset;
            b[2] = center[2] + r * sin(rads*(j+1));
        
            drawLine(a,b,color,thickness);
        }
        
        for(int j=0; j<numSections; j++)
        {
            a[0] = center[0] + offset;
            a[1] = center[1] + r * cos(rads*j);
            a[2] = center[2] + r * sin(rads*j);
        
            b[0] = center[0] + offset;
            b[1] = center[1] + r * cos(rads*(j+1));
            b[2] = center[2] + r * sin(rads*(j+1));
        
            drawLine(a,b,color,thickness);
        }
        
    }
}

inline void Renderer::flush()
{
    glFlush();
}

inline void Renderer::setCameraPosition(const Vector3& eye)
{
    glMatrixMode( GL_MODELVIEW );
    glLoadMatrixf(mv);
    glTranslatef(-eye[0], -eye[1], -eye[2]);    
}


inline void Renderer::setCameraLookAt(const Vector3& eye, const Vector3& target, const Vector3& refUp)
{
    Vector3 forward;
    Vector3 up = refUp;
    up.normalize();
    
    forward = target - eye;
    forward.normalize();

    Vector3 side = Vector3::crossProduct(forward,up); //compute side
    up = Vector3::crossProduct(side,forward);  //compute orthogonal up
    
    mv[0] = side[0];
    mv[1] = up[0];
    mv[2] = -forward[0];
    mv[3] = 0.f;

    mv[4] = side[1];
    mv[5] = up[1];
    mv[6] = -forward[1];
    mv[7] = 0.f;

    mv[8] = side[2];
    mv[9] = up[2];
    mv[10] = -forward[2];
    mv[11] = 0.f;

    mv[12] = 0;
    mv[13] = 0;
    mv[14] = 0;
    mv[15] = 1.f;
    
    glMatrixMode( GL_MODELVIEW );
    glLoadMatrixf(mv);
    glTranslatef(-eye[0], -eye[1], -eye[2]);
}



inline void Renderer::setCameraLense(float angleFOV, float nearPlane, float farPlane)
{
    const float ratio = (width > 0.f && height > 0.f) ? width / height : 1.f;
    const float size = tanf(anglesToRadians(angleFOV) / 2.f);
    
    float right = size;
    float top = size / ratio;
    
    projection[0] = nearPlane/right;
    projection[1] = 0.f;
    projection[2] = 0.f;
    projection[3] = 0.f;
        
    projection[4] = 0.f;
    projection[5] = nearPlane/top;
    projection[6] = 0.f;
    projection[7] = 0.f;
    
    projection[8] = 0.f;
    projection[9] = 0.f;
    projection[10] = -(farPlane+nearPlane) / (farPlane - nearPlane);
    projection[14] = -2.f*farPlane*nearPlane / (farPlane - nearPlane);
        
    projection[12] = 0.f;
    projection[13] = 0.f;
    projection[11] = -1.f;
    projection[15] = 0.f;
    
    glMatrixMode( GL_PROJECTION );
    glLoadMatrixf(projection);
}

#endif
