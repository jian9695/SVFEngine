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

PointsRenderer* PointsRenderer::create()
{
	PointsRenderer* pointsRender = new PointsRenderer;
	osg::ref_ptr<osg::Geode> points = new osg::Geode;
	osg::ref_ptr <osg::Geode> texts = new osg::Geode;
	osg::ref_ptr<osg::Program> program = new osg::Program;

	char fragmentShaderSource[] =
		"void main(void) \n"
		"{\n"
		"     gl_FragColor = vec4(1,0,0,1);\n"
		"}\n";

	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
	osg::StateSet* ss = points->getOrCreateStateSet();
	ss->setAttribute(program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	points->setCullingActive(false);
	osg::ref_ptr<osg::Point> point = new osg::Point(5.0);
	point->setDistanceAttenuation(osg::Vec3(0.0, 0.0000, 0.05f));
	ss->setAttribute(point.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setRenderBinDetails(6000, "RenderBin");
	//ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	//ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	texts->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	
	pointsRender->addChild(points.get());
	pointsRender->addChild(texts.get());
	//pointsRender->addChild(lod.get());
	return pointsRender;
}

void PointsRenderer::pushPoint(const osg::Vec3d& point, const std::string& label)
{
	if (getNumChildren() < 2)
		return;
	osg::Geode* points = dynamic_cast<osg::Geode*> (getChild(0));
	osg::Geode* texts = dynamic_cast<osg::Geode*> (getChild(1));
	if (!points || !texts)
		return;

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry;
	vertices->push_back(point);
	polyGeom->setVertexArray(vertices.get());
	polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
	osg::ref_ptr<osg::Geode> pointGeo = new osg::Geode;
	//pointGeo->addDrawable(polyGeom.get());

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
	lod->addChild(text.get(), 0, 1000);
	//lod->addChild(pointGeo.get(), 100, 1000);

	points->addDrawable(polyGeom.get());
	texts->addChild(lod.get());
}

void PointsRenderer::popPoint()
{
	if (getNumChildren() < 2)
		return;
	osg::Geode* points = dynamic_cast<osg::Geode*> (getChild(0));
	osg::Geode* texts = dynamic_cast<osg::Geode*> (getChild(1));
	if (!points || !texts)
		return;
	if (points->getNumDrawables() < 1 || texts->getNumDrawables() < 1)
		return;
	points->removeDrawable(points->getDrawable(points->getNumDrawables() - 1));
	texts->removeDrawable(texts->getDrawable(texts->getNumDrawables() - 1));
}