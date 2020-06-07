#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include "PointsRenderer.h"
#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/Texture2D>
#include <osg/PointSprite>
#include <osg/PolygonMode>
#include <osg/Geometry>
#include <osgText/Font>
#include <osgText/Text>

void PointsRenderer::pushPoint(const osg::Vec3d& point, const std::string& label)
{
	if (!m_pointShader)
	{
		m_pointShader = new osg::Program;
		char vertexShaderSource[] =
			"void main(void)\n"
			"{\n"
			"   gl_PointSize = 5;\n"
			"   gl_Position = gl_ModelViewProjectionMatrix *  gl_Vertex;\n"
			"}\n";
		char fragmentShaderSource[] =
			"void main(void) \n"
			"{\n"
			"     gl_FragColor = vec4(1,0,0,1);\n"
			"}\n";
		m_pointShader->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
		m_pointShader->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
		getOrCreateStateSet()->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		getOrCreateStateSet()->setRenderBinDetails(6000, "RenderBin");
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
	}

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry;
	vertices->push_back(point);
	polyGeom->setVertexArray(vertices.get());
	polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
	polyGeom->getOrCreateStateSet()->setAttribute(m_pointShader, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	
	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setColor(osg::Vec4(1,1,0,1));
	text->setFont("fonts/times.ttf");
	text->setCharacterSize(20);
	text->setPosition(point);
	text->setAxisAlignment(osgText::Text::SCREEN);
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	text->setText(label);

	osg::ref_ptr <TextLOD> lod = new TextLOD(point);
	lod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
	lod->addChild(polyGeom.get(), 0, 1000);
	lod->addChild(text.get(), 1000, 10000000000);
	addChild(lod.get());
}

void PointsRenderer::popPoint()
{
	if (getNumDrawables() < 2)
		return;
	removeDrawable(getDrawable(getNumDrawables() - 1));
	removeDrawable(getDrawable(getNumDrawables() - 1));
}