
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <osgEarthUtil/ExampleResources>

#include "CustomControls.h"
#include "GrassSolar.h"
#include "SolarInteractiveHandler.h"
#include "ModelLoader.h"

using namespace osgEarth;
using namespace osgEarth::Util;
//using namespace osgEarth::Drivers;
//using namespace osgEarth::Features;
//using namespace osgEarth::Symbology;

const int UI_FONT_SIZE = 18;
SolarParam m_solarParam;
size_t m_frameCount = 1;

osg::ref_ptr<SolarInteractiveHandler> m_skyViewHandler;
CustomControls::HBox* m_popupControl;
std::map<std::string, CustomControls::Control*> m_controls;
CustomControls::VBox* m_parametersControl;
CustomControls::VBox* m_fisheyeControl;
CustomControls::VBox* m_resultLabelsControl = new CustomControls::VBox();
CustomControls::LabelControl* m_svfLabel;
CustomControls::LabelControl* m_globalRadLabel;
CustomControls::LabelControl* m_beamRadLabel;
CustomControls::LabelControl* m_diffuseRadLabel;

SolarParam createSolarParam()
{
  SolarParam param;
  param.m_aspect = 270;
  param.m_slope = 0;
  param.m_lon = -9999;
  param.m_lat = 37.5131;
  param.m_day = 183;
  param.m_time_step = 1;
  param.m_linke = 3.0;
  param.m_startDay = param.m_day;
  param.m_endDay = param.m_day;
  param.m_isSingleDay = true;
  param.m_isInstantaneous;
  param.m_elev = 0;
  return param;
}

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

bool getSingleDayMode() 
{
  auto iter = m_controls.find("SingleDayMode");
  return ((ParamControlBase*)iter->second)->m_check->getValue();
}

float getStartDay()
{
  auto iter = m_controls.find("StartDay");
  return ((ParamControlBase*)iter->second)->m_slider->getValue();
}

float getEndDay()
{
  auto iter = m_controls.find("EndDay");
  return ((ParamControlBase*)iter->second)->m_slider->getValue();
}

void onResultsUpdated(float svf, SolarRadiation rad)
{
  m_svfLabel->setText("SVF: " + Utils::value2String(svf, 3));
  std::string unit = " [kWh/m2]";
  rad = rad / 1000;
  m_globalRadLabel->setText("Global radiation: " + Utils::value2String(rad.m_global, 3) + unit);
  m_beamRadLabel->setText("Beam radiation: " + Utils::value2String(rad.m_beam, 3) + unit);
  m_diffuseRadLabel->setText("Diffuse radiation: " + Utils::value2String(rad.m_diffuse, 3) + unit);
  //printf("Global: %f\n", rad.global);
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
      str = Utils::padRight(str, 7, '0');
    }
    return str;
  }

  void onValueChanged(CustomControls::Control* control, float value)
  {
    int intValue = (int)value;
    bool isSingleMode = m_solarParam.m_isSingleDay;
    if (m_name == "StartDay")
    {
      m_solarParam.m_startDay = intValue;
      if (isSingleMode || m_solarParam.m_endDay < intValue)
      {
        auto iter = m_controls.find("EndDay");
        ParamControl* endSlider = (ParamControl*)iter->second;
        endSlider->m_slider->setValue(value);
        m_solarParam.m_endDay = intValue;
      }
    }
    else if (m_name == "EndDay")
    {
      m_solarParam.m_endDay = intValue;
      if (isSingleMode || m_solarParam.m_startDay > intValue)
      {
        auto iter = m_controls.find("StartDay");
        ParamControl* endSlider = (ParamControl*)iter->second;
        endSlider->m_slider->setValue(value);
        m_solarParam.m_startDay = intValue;
      }
    }
    else if (m_name == "Latitude")
    {
      auto iter = m_controls.find("Latitude");
      ParamControl* slider = (ParamControl*)iter->second;
      slider->m_slider->setValue(value);
      m_solarParam.m_lat = value;
    }
    else if (m_name == "Elevation")
    {
      auto iter = m_controls.find("Elevation");
      ParamControl* slider = (ParamControl*)iter->second;
      slider->m_slider->setValue(value);
      m_solarParam.m_elev = value;
    }
    else if (m_name == "TimeStep")
    {
      m_solarParam.m_time_step = value;
    }

    if (m_isInteger)
      value = intValue;
    m_valueLabel->setText(toString(value));
  }

  void onValueChanged(CustomControls::Control* control, bool value)
  {
    m_solarParam.m_isSingleDay = value;
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
      m_solarParam.m_isSingleDay = value;
    }
    else if (m_name == "Latitude")
    {
      m_solarParam.m_useLatitudeOverride = value;
    }
    else if (m_name == "Elevation")
    {
      m_solarParam.m_useElevationOverride = value;
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

class PopupControl : public CustomControls::HBox
{
public:
  PopupControl() :CustomControls::HBox()
  {
    m_point.m_id = -1;
    setChildSpacing(0);
    setChildVertAlign(CustomControls::Control::ALIGN_CENTER);
    setHorizFill(true);
    m_nameLabel = new CustomControls::LabelControl("");
    m_nameLabel->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_nameLabel->setVertAlign(CustomControls::Control::ALIGN_CENTER);
    m_nameLabel->setTextBackdropOffset(3);
    m_nameLabel->setFontSize(UI_FONT_SIZE);
    addControl(m_nameLabel);
    m_fisheyeImagel = new CustomControls::ImageControl;
    m_fisheyeImagel->setSize(256, 256);
    osg::Vec4 borderColor(0.0, 0.0, 0.0, 1.0);
    m_fisheyeImagel->setBorderColor(borderColor);
    addControl(m_fisheyeImagel);
  }

  void SetPoint(SolarRadiationPoint& point)
  {
    m_point = point;
    m_nameLabel->setText(m_point.toString());
    osg::Image* fisheye = m_skyViewHandler->getFisheyeForPoint(m_point.m_id);
    if (fisheye)
      m_fisheyeImagel->setImage(fisheye);
  }

  SolarRadiationPoint m_point;
  CustomControls::LabelControl* m_nameLabel;
  CustomControls::ImageControl* m_fisheyeImagel;
};

void createMainUIControls(CustomControls::ControlCanvas* cs)
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
  CustomControls::HBox* linkieSlider = new ParamControl("Linkie", Utils::padRight("Linkie", maxLabelLen), 3, 8, m_solarParam.m_linke, true);
  CustomControls::HBox* startDaySlider = new ParamControl("StartDay", Utils::padRight("Start Day", maxLabelLen), 1, 365, m_solarParam.m_startDay, true);
  CustomControls::HBox* endDaySlider = new ParamControl("EndDay", Utils::padRight("End Day", maxLabelLen), 1, 365, m_solarParam.m_endDay, true);
  CustomControls::HBox* isSingleDayCheck = new ParamControl("SingleDayMode", Utils::padRight("Single Day Mode", maxLabelLen), m_solarParam.m_isSingleDay);
  CustomControls::HBox* timeStepSlider = new ParamControl("TimeStep", Utils::padRight("Time Step (hours)", maxLabelLen), 0.1, 1, m_solarParam.m_time_step, false);
  CustomControls::HBox* latSlider = new ParamControl("Latitude", Utils::padRight("Default Latitude", maxLabelLen), -90, 90, m_solarParam.m_lat, false, true);
  CustomControls::HBox* elevSlider = new ParamControl("Elevation", Utils::padRight("Base Elevation", maxLabelLen), 0, 9999, m_solarParam.m_elev, false, true);
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
  m_fisheyeControl->setBackColor(backgroundColor);
  m_fisheyeControl->setBorderColor(borderColor);
  CustomControls::ImageControl* fishEyeImg = new CustomControls::ImageControl(m_skyViewHandler->fisheyeSurface()->Texture());
  fishEyeImg->setSize(320, 320);
  m_fisheyeControl->addControl(fishEyeImg);
  ul->addControl(togglesControl);
  ul->addControl(m_parametersControl);
  ul->addControl(m_resultLabelsControl);
  ul->addControl(m_fisheyeControl);

  cs->addControl(ul);
}

void createPopup(CustomControls::ControlCanvas* cs)
{
  PopupControl* popup = new PopupControl();
  popup->setPosition(200, 200);
  popup->setPadding(5);
  int maxLabelLen = 20;
  osg::Vec4 backgroundColor(0.0, 0.0, 0.0, 0.6);
  osg::Vec4 borderColor(1.0, 1.0, 1.0, 1.0);
  osg::Vec4 fontColor(1, 1, 1, 1);
  popup->setBorderColor(borderColor);
  popup->setBackColor(backgroundColor);
  popup->setBorderWidth(5);
  popup->m_nameLabel->setFontSize(UI_FONT_SIZE);
  popup->m_nameLabel->setForeColor(fontColor);
  popup->setNodeMask(false);
  m_popupControl = popup;
  cs->addControl(popup);
}

class MainUIEventHandler : public osgGA::GUIEventHandler
{
private:

public:
  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
      return false;
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
      return false;
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
      return false;
    if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK)
      return false;
    if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
      return false;

    int key = ea.getUnmodifiedKey();
    PopupControl* popup = (PopupControl*)m_popupControl;
    osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
    if (!viewer)
      return false;

    if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME & m_frameCount % 10 == 0)
    {
      SolarRadiationPoint point;
      if (m_skyViewHandler->queryPoint(ea.getXnormalized(), ea.getYnormalized(), point))
      {
        float viewWidth = viewer->getCamera()->getViewport()->width();
        float viewHeight = viewer->getCamera()->getViewport()->height();
        int x = (int)((ea.getXnormalized() * 0.5 + 0.5) * viewWidth) + 10;
        int y = (int)((1.0 - (ea.getYnormalized() * 0.5 + 0.5)) * viewHeight) + 10;
        popup->setPosition(x, y);
        if (popup->m_point.m_id != point.m_id)
        {
          popup->SetPoint(point);
        }
        popup->setNodeMask(true);
      }
      else
      {
        popup->setNodeMask(false);
      }
    }
    return false;
  }
};

int main(int argc, char** argv)
{
  m_solarParam = createSolarParam();

  osg::ArgumentParser arguments(&argc, argv);

  if (argc < 2)
    return 0;

  // create a viewer:
  osgViewer::Viewer viewer(arguments);

  // Tell the database pager to not modify the unref settings
  viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, false);

  // thread-safe initialization of the OSG wrapper manager. Calling this here
  // prevents the "unsupported wrapper" messages from OSG
  osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");
 
  osg::ref_ptr<osg::Node> scene = MapNodeHelper().load(arguments, &viewer);
  if (!scene)
  {
    scene = osgDB::readNodeFiles(arguments);
  }

  if (!scene)
  {
    scene = ModelLoader::Load3DTiles(arguments[1]);
  }

  if (!scene)
    return 0;

  osg::ref_ptr<osg::Group> root = new osg::Group;
  root->addChild(scene.get());

  viewer.setSceneData(root.get());

  MapNode* mapNode = MapNode::findMapNode(scene);

  osg::ref_ptr<osgGA::CameraManipulator> manip;
  if (mapNode)
  { 
    // install our default manipulator (do this before calling load)
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
    if (!Utils::nodeHasNormals(scene))
    {
      viewer.getCamera()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }
  }

  viewer.setCameraManipulator(manip.get());

  m_skyViewHandler = new SolarInteractiveHandler(scene, root, mapNode, manip, &viewer, &m_solarParam, onResultsUpdated);
 
  // create a surface to house the controls
  CustomControls::ControlCanvas* uiCanvas = CustomControls::ControlCanvas::getOrCreate(&viewer);
  // create some controls.
  createMainUIControls(uiCanvas);

  // create a surface to house the controls
  CustomControls::ControlCanvas* popupCanvas = CustomControls::ControlCanvas::getOrCreate(&viewer);
  // create some controls.
  createPopup(popupCanvas);

  // zoom to a good startup position

  viewer.addEventHandler(m_skyViewHandler);
  viewer.addEventHandler(new MainUIEventHandler);
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
  //viewer.setUpViewInWindow(50, 50, 1024, 768);
  viewer.realize();
  while (!viewer.done())
  {
    viewer.frame();
    if (m_frameCount % 10 == 0)
    {
      m_skyViewHandler->postDrawUpdate();
    }
    m_frameCount++;
    if (m_frameCount > 1000000)
      m_frameCount = 1;
  }
  return 0;
}
