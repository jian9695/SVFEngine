
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif
#include "SolarInteractiveHandler.h"
#include "ShapeFile.h"
#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/LogarithmicDepthBuffer>
#include <osgEarthFeatures/FeatureModelLayer>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/engine_rex/RexTerrainEngineOptions>
#include <osgEarth/Cache>
#include <osgEarthDrivers/cache_filesystem/FileSystemCache>
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgEarth/Registry>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>
#include "CustomControls.h"
#include "GrassSolar.h"
#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <iostream>


using namespace osgEarth::Symbology;
using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

SolarParam m_solarParam;
osg::ref_ptr<SolarInteractiveHandler> m_skyViewHandler;
const int UI_FONT_SIZE = 18;

SolarParam createSolarParam()
{
  SolarParam param;
  param.aspect = 270;
  param.slope = 0;
  param.lon = 122.1204;
  param.lat = 37.5131;
  param.day = 183;
  param.time_step = 1;
  param.linke = 3.0;
  param.startDay = param.day;
  param.endDay = param.day;
  param.isSingleDay = true;
  param.isInstantaneous;
  param.elev = 0;
  return param;
}

void createControls(CustomControls::ControlCanvas*);
CustomControls::ImageControl* s_imageControl = 0L;

std::map<std::string, CustomControls::Control*> m_controls;
CustomControls::VBox* m_parametersControl;
CustomControls::VBox* m_fisheyeControl;
CustomControls::VBox* m_resultLabelsControl = new CustomControls::VBox();
CustomControls::LabelControl* m_svfLabel;
CustomControls::LabelControl* m_globalRadLabel;
CustomControls::LabelControl* m_beamRadLabel;
CustomControls::LabelControl* m_diffuseRadLabel;

class ParamControlBase : public CustomControls::HBox
{
public:
  ParamControlBase(std::string name)
    :CustomControls::HBox()
  {
    m_name = name;
    setName(name);
    m_controls[m_name] = (CustomControls::Control*)this;
  }
  CustomControls::HSliderControl* m_slider;
  CustomControls::CheckBoxControl* m_check;
  CustomControls::LabelControl* m_nameLabel;
  std::string m_name;

  virtual void onValueChanged(CustomControls::Control* control, float value) {}

  virtual void onValueChanged(CustomControls::Control* control, bool value) {}
};

bool GetSingleDayMode() 
{
  auto iter = m_controls.find("SingleDayMode");
  return ((ParamControlBase*)iter->second)->m_check->getValue();
}

float GetStartDay()
{
  auto iter = m_controls.find("StartDay");
  return ((ParamControlBase*)iter->second)->m_slider->getValue();
}

float GetEndDay()
{
  auto iter = m_controls.find("EndDay");
  return ((ParamControlBase*)iter->second)->m_slider->getValue();
}

std::string PadRight(std::string str, const size_t num, const char paddingChar = ' ')
{
  if (num > str.size())
    str.insert(str.size(), num - str.size(), paddingChar);
  return str;
}

void ResultsUpdated(float svf, SolarRadiation rad)
{
  m_svfLabel->setText("SVF: " + Utils::value2String(svf, 3));
  std::string unit = " [kWh/m2]";
  rad = rad / 1000;
  m_globalRadLabel->setText("Global radiation: " + Utils::value2String(rad.global, 3) + unit);
  m_beamRadLabel->setText("Beam radiation: " + Utils::value2String(rad.beam, 3) + unit);
  m_diffuseRadLabel->setText("Diffuse radiation: " + Utils::value2String(rad.diffuse, 3) + unit);
  printf("Global: %f\n", rad.global);
}

class ParamControlEventHandler : public CustomControls::ControlEventHandler
{
public:
  ParamControlBase* _paramControl;
  ParamControlEventHandler(ParamControlBase* paramControl) :
    _paramControl(paramControl)
  {

  }
  virtual void onValueChanged(CustomControls::Control* control, float value)
  {
    if (_paramControl)
      _paramControl->onValueChanged(control, value);
  }

  virtual void onValueChanged(CustomControls::Control* control, bool value)
  {
    if (_paramControl)
      _paramControl->onValueChanged(control, value);
  }
};

class ParamControl : public ParamControlBase, public CustomControls::ControlEventHandler
{
public:

  void Init(std::string name, std::string label, bool isInteger)
  {
    m_isInteger = isInteger;
    setChildSpacing(0);
    setChildVertAlign(CustomControls::Control::ALIGN_CENTER);
    setHorizFill(true);
    m_nameLabel = new CustomControls::LabelControl(label);
    m_nameLabel->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_nameLabel->setVertAlign(CustomControls::Control::ALIGN_CENTER);
    m_nameLabel->setTextBackdropOffset(3);
    m_nameLabel->setFontSize(UI_FONT_SIZE);
    addControl(m_nameLabel);

  }
  ParamControl(std::string name, std::string label, float min, float max, float value, bool isInteger)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    Init(name, label, isInteger);

    m_slider = new CustomControls::HSliderControl(min, max, value);
    //m_slider->setBackColor(.6, 0, 0, 1);
    m_slider->setHeight(UI_FONT_SIZE);
    m_slider->setWidth(200);
    m_slider->setName(name);
    m_slider->addEventHandler(new ParamControlEventHandler(this));
    m_slider->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_valueLabel = new CustomControls::LabelControl(name);
    m_valueLabel->setText(toString(value));
    addControl(m_slider);
    addControl(m_valueLabel);
  }

  ParamControl(std::string name, std::string label, bool isChecked)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    Init(name, label, false);

    m_check = new CustomControls::CheckBoxControl(isChecked);
    //slider->setBackColor(.6, 0, 0, 1);
    m_check->setHeight(10);
    m_check->setName(name);
    m_check->addEventHandler(new ParamControlEventHandler(this));
    addControl(m_check);
  }

  ParamControl(std::string name, std::string label, float min, float max, float value, bool isInteger, bool isChecked)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    Init(name, label, isInteger);

    m_slider = new CustomControls::HSliderControl(min, max, value);
    m_slider->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    //m_slider->setBackColor(.6, 0, 0, 1);
    m_slider->setHeight(10);
    m_slider->setWidth(200);
    m_slider->setName(name);
    m_slider->addEventHandler(new ParamControlEventHandler(this));
    m_valueLabel = new CustomControls::LabelControl(name);
    m_valueLabel->setText(toString(value));
    m_valueLabel->setFontSize(UI_FONT_SIZE);
    m_check = new CustomControls::CheckBoxControl(isChecked);
    //slider->setBackColor(.6, 0, 0, 1);
    m_check->setHeight(10);
    m_check->setName(name);
    m_check->addEventHandler(new ParamControlEventHandler(this));
    addControl(m_slider);
    addControl(m_valueLabel);
    addControl(m_check);
  }

private:
  CustomControls::LabelControl* m_valueLabel;
  bool m_isInteger;
  std::string toString(float value)
  {
    //bool isInt = (floor(value) == value);
    std::stringstream buf;
    std::string str = "";
    if (m_isInteger)
    {
      buf << (int)value;
      str = buf.str();
    }
    else
    {
      buf.precision(2);
      buf << std::fixed << value;
      str = buf.str();
      str = PadRight(str, 7, '0');
    }
    return str;
  }

  void onValueChanged(CustomControls::Control* control, float value)
  {
    int intValue = (int)value;
    bool isSingleMode = m_solarParam.isSingleDay;
    if (m_name == "StartDay")
    {
      m_solarParam.startDay = intValue;
      if (isSingleMode || m_solarParam.endDay < intValue)
      {
        auto iter = m_controls.find("EndDay");
        ParamControl* endSlider = (ParamControl*)iter->second;
        endSlider->m_slider->setValue(value);
        m_solarParam.endDay = intValue;
      }
    }
    else if (m_name == "EndDay")
    {
      m_solarParam.endDay = intValue;
      if (isSingleMode || m_solarParam.startDay > intValue)
      {
        auto iter = m_controls.find("StartDay");
        ParamControl* endSlider = (ParamControl*)iter->second;
        endSlider->m_slider->setValue(value);
        m_solarParam.startDay = intValue;
      }
    }
    else if (m_name == "Latitude")
    {
      auto iter = m_controls.find("Latitude");
      ParamControl* slider = (ParamControl*)iter->second;
      slider->m_slider->setValue(value);
      m_solarParam.lat = value;
    }
    else if (m_name == "Elevation")
    {
      auto iter = m_controls.find("Elevation");
      ParamControl* slider = (ParamControl*)iter->second;
      slider->m_slider->setValue(value);
      m_solarParam.elev = value;
    }
    else if (m_name == "TimeStep")
    {
      m_solarParam.time_step = value;
    }

    if (m_isInteger)
      value = intValue;
    m_valueLabel->setText(toString(value));
  }

  void onValueChanged(CustomControls::Control* control, bool value)
  {
    m_solarParam.isSingleDay = value;
    if (m_name == "SingleDayMode")
    {
      auto iter = m_controls.find("StartDay");
      ParamControl* starSlider = (ParamControl*)iter->second;
      iter = m_controls.find("EndDay");
      ParamControl* endSlider = (ParamControl*)iter->second;
      if (value)
      {
        endSlider->m_slider->setValue(starSlider->m_slider->getValue());
      }
      m_solarParam.isSingleDay = value;
    }
    else if (m_name == "Latitude")
    {
      m_solarParam.useLatitudeOverride = value;
    }
    else if (m_name == "Elevation")
    {
      m_solarParam.useElevationOverride = value;
    }
    else if (m_name == "ToggleParameters")
    {
      m_parametersControl->setNodeMask(value);
    }
    else if (m_name == "ToggleResults")
    {
      m_resultLabelsControl->setNodeMask(value);
    }
    else if (m_name == "ToggleFisheye")
    {
      m_fisheyeControl->setNodeMask(value);
    }
  }
};

void createControls(CustomControls::ControlCanvas* cs)
{
  CustomControls::VBox* ul = new CustomControls::VBox();
  ul->setPosition(0, 0);
  ul->setPadding(5);
  int maxLabelLen = 20;
  osg::Vec4 backgroundColor(0.0, 0.0, 0.0, 0.6);
  osg::Vec4 borderColor(0.0, 0.0, 0.0, 1.0);
  osg::Vec4 fontColor(1, 1, 1, 1);

  CustomControls::VBox* togglesControl = new CustomControls::VBox();
  CustomControls::HBox* toggleParameters = new ParamControl("ToggleParameters", "Toggle Hide/Show Parameters", true);
  CustomControls::HBox* toggleResults = new ParamControl("ToggleResults", "Toggle Hide/Show Results", true);
  CustomControls::HBox* toggleFisheye = new ParamControl("ToggleFisheye", "Toggle Hide/Show Fisheye", true);
  togglesControl->addControl(toggleParameters);
  togglesControl->addControl(toggleResults);
  togglesControl->addControl(toggleFisheye);
  togglesControl->setBackColor(backgroundColor);
  togglesControl->setBorderColor(borderColor);

  m_parametersControl = new CustomControls::VBox();
  m_parametersControl->setBackColor(backgroundColor);
  m_parametersControl->setBorderColor(borderColor);
  CustomControls::LabelControl* titleLabel = new CustomControls::LabelControl("GRASS GIS r.sun parameters", fontColor);
  titleLabel->setFontSize(UI_FONT_SIZE);
  CustomControls::HBox* linkieSlider = new ParamControl("Linkie", PadRight("Linkie", maxLabelLen), 3, 8, m_solarParam.linke, true);
  CustomControls::HBox* startDaySlider = new ParamControl("StartDay", PadRight("Start Day", maxLabelLen), 1, 365, m_solarParam.startDay, true);
  CustomControls::HBox* endDaySlider = new ParamControl("EndDay", PadRight("End Day", maxLabelLen), 1, 365, m_solarParam.endDay, true);
  CustomControls::HBox* isSingleDayCheck = new ParamControl("SingleDayMode", PadRight("Single Day Mode", maxLabelLen), m_solarParam.isSingleDay);
  CustomControls::HBox* timeStepSlider = new ParamControl("TimeStep", PadRight("Time Step (hours)", maxLabelLen), 0.1, 1, m_solarParam.time_step, false);
  CustomControls::HBox* latSlider = new ParamControl("Latitude", PadRight("Default Latitude", maxLabelLen), -90, 90, m_solarParam.lat, false, true);
  CustomControls::HBox* elevSlider = new ParamControl("Elevation", PadRight("Base Elevation", maxLabelLen), 0, 9999, m_solarParam.elev, false, true);
  m_parametersControl->addControl(titleLabel);
  m_parametersControl->addControl(isSingleDayCheck);
  m_parametersControl->addControl(startDaySlider);
  m_parametersControl->addControl(endDaySlider);
  m_parametersControl->addControl(latSlider);
  m_parametersControl->addControl(elevSlider);
  m_parametersControl->addControl(timeStepSlider);
  m_parametersControl->addControl(linkieSlider);

  m_resultLabelsControl = new CustomControls::VBox();
  m_resultLabelsControl->setBackColor(backgroundColor);
  m_resultLabelsControl->setBorderColor(borderColor);
  m_resultLabelsControl->setWidth(320);

  m_svfLabel = new CustomControls::LabelControl("SVF:", fontColor, UI_FONT_SIZE);
  m_globalRadLabel = new CustomControls::LabelControl("Global radiation:", fontColor, UI_FONT_SIZE);
  m_beamRadLabel = new CustomControls::LabelControl("Beam radiation:", fontColor, UI_FONT_SIZE);
  m_diffuseRadLabel = new CustomControls::LabelControl("Diffuse radiation:", fontColor, UI_FONT_SIZE);

  m_resultLabelsControl->addControl(m_svfLabel);
  m_resultLabelsControl->addControl(m_globalRadLabel);
  m_resultLabelsControl->addControl(m_beamRadLabel);
  m_resultLabelsControl->addControl(m_diffuseRadLabel);

  m_fisheyeControl = new CustomControls::VBox();
  //m_fisheyeControl->setBackColor(backgroundColor);
  m_fisheyeControl->setBorderColor(borderColor);
  CustomControls::ImageControl* fishEyeImg = new CustomControls::ImageControl(m_skyViewHandler->FisheyeSurface()->Texture());
  fishEyeImg->setSize(320, 320);
  m_fisheyeControl->addControl(fishEyeImg);
  ul->addControl(togglesControl);
  ul->addControl(m_parametersControl);
  ul->addControl(m_resultLabelsControl);
  ul->addControl(m_fisheyeControl);

  cs->addControl(ul);
}

int main(int argc, char** argv)
{
  m_solarParam = createSolarParam();
  GDALAllRegister();

  osg::ArgumentParser arguments(&argc, argv);

  // help?
  if (arguments.read("--help"))
    return 0;

  // create a viewer:
  osgViewer::Viewer viewer(arguments);

  // Tell the database pager to not modify the unref settings
  viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, false);

  // thread-safe initialization of the OSG wrapper manager. Calling this here
  // prevents the "unsupported wrapper" messages from OSG
  osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

  // load an earth file, and support all or our example command-line options
  // and earth file <external> tags    
  osg::ref_ptr<osg::Node> scene = MapNodeHelper().load(arguments, &viewer);
  if (!scene)
  {
    scene = osgDB::readNodeFiles(arguments);
  }

  osg::ref_ptr<osg::Group> root = new osg::Group;
  root->addChild(scene.get());

  viewer.setSceneData(root.get());

  MapNode* mapNode = MapNode::findMapNode(scene);
  // install our default manipulator (do this before calling load)
  osg::ref_ptr<osgGA::CameraManipulator> manip;
  if (mapNode)
  {
    manip = new EarthManipulator(arguments);

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);
  }
  else
  {
    manip = new osgGA::TrackballManipulator();
  }

  viewer.setCameraManipulator(manip.get());

  m_skyViewHandler = new SolarInteractiveHandler(scene, root, mapNode, manip, &viewer, &m_solarParam, ResultsUpdated);
 
  // create a surface to house the controls
  CustomControls::ControlCanvas* cs = CustomControls::ControlCanvas::getOrCreate(&viewer);

  // create some controls.
  createControls(cs);

  // zoom to a good startup position

  viewer.addEventHandler(m_skyViewHandler);

  viewer.addEventHandler(new osgViewer::ThreadingHandler);

  // add the window size toggle handler
  viewer.addEventHandler(new osgViewer::WindowSizeHandler);

  // add the stats handler
  viewer.addEventHandler(new osgViewer::StatsHandler);

  // add the LOD Scale handler
  viewer.addEventHandler(new osgViewer::LODScaleHandler);

  // add the state manipulator
  viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

  unsigned int width, height;
  osg::GraphicsContext::ScreenIdentifier main_screen_id;
  osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
  main_screen_id.readDISPLAY();
  main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
  wsi->getScreenResolution(main_screen_id, width, height);

  printf("%d,%d\n", width, height);
  //viewer.getCamera()->setViewport(50, 50, 1600, 1024);
  if (mapNode)
  {
    Metrics::run(viewer);
    return 0;
  }
  return viewer.run();
}
