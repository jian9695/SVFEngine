
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Depth>

#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <osgEarthUtil/ExampleResources>

#include "CustomControls.h"
#include "GrassSolar.h"
#include "SolarInteractiveHandler.h"
#include "ModelLoader.h"
#include "ShadowCaster.h"
#include "GDAL_DS.h"

using namespace osgEarth;
using namespace osgEarth::Util;
//using namespace osgEarth::Drivers;
//using namespace osgEarth::Features;
//using namespace osgEarth::Symbology;

const int UI_FONT_SIZE = 18;
SolarParam m_solarParam;
size_t m_frameCount = 1;

osg::ref_ptr<SolarInteractiveHandler> m_solarInteractiveHandler;
CustomControls::VBox* m_mainUIControl;
CustomControls::HBox* m_popupControl;
std::map<std::string, CustomControls::Control*> m_controls;
CustomControls::VBox* m_parametersControl;
CustomControls::VBox* m_fisheyeControl;
CustomControls::VBox* m_resultLabelsControl = new CustomControls::VBox();
CustomControls::LabelControl* m_svfLabel;
CustomControls::LabelControl* m_globalRadLabel;
CustomControls::LabelControl* m_beamRadLabel;
CustomControls::LabelControl* m_diffuseRadLabel;
CustomControls::LabelControl* m_statusBar;
MapNode* m_mapNode;
CustomControls::CustomImageControl* m_compass;

CustomControls::CustomImageControl* createCompass(CustomControls::ControlCanvas* cs, int viewWidth, int viewHeight)
{
  char fragmentSource[] =
    "uniform sampler2D texture0;\n"
    "uniform float rotateAngle;\n"
    "vec2 rotateXY(vec2 xy, float rotation)\n"
    "{\n"
    "   float mid = 0.5;\n"
    "   float x = cos(rotation) * (xy.x - mid) + sin(rotation) * (xy.y - mid) + mid;\n"
    "   float y = cos(rotation) * (xy.y - mid) - sin(rotation) * (xy.x - mid) + mid;\n"
    "   return vec2(x,y);\n"
    "}\n"
    "void main(void) \n"
    "{\n"
    "    vec4 color = texture2D(texture0, rotateXY(gl_TexCoord[0].xy, rotateAngle * 0.0174533));\n"
    "    if(color.a < 0.5)\n"
    "      color = vec4(0.1,0.1,0.1,0.4);\n"
    "    else\n"
    "      color = color + vec4(0,0.3,0,0);\n"
    "    gl_FragColor = color;\n"
    "}\n";
  osg::ref_ptr<osg::Image> img = osgDB::readImageFile("./data/compass.png");
  osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
  tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
  tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
  tex->setImage(img.get());

  CustomControls::CustomImageControl* imageCtrl = new CustomControls::CustomImageControl;
  imageCtrl->setSize(128, 128);
  imageCtrl->setPosition(viewWidth - 128 - 10, 10);
  imageCtrl->setImage(img.get());
  imageCtrl->getOrCreateStateSet()->addUniform(new osg::Uniform("rotateAngle", 0.0f));
  
  ProgramBinder binder;
  binder.initialize("NorthArrowProgram", imageCtrl->getOrCreateStateSet());
  binder.setFragmentShader(fragmentSource);
  imageCtrl->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
  cs->addChild(imageCtrl);
  return imageCtrl;
}

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

void onResultsUpdated(float svf, SolarRadiation rad)
{
  m_svfLabel->setText("SVF: " + Utils::value2String(svf, 3));
  rad = rad / 1000;
  m_globalRadLabel->setText("Global radiation [kWh/m2]: " + Utils::value2String(rad.m_global, 3));
  m_beamRadLabel->setText("Beam radiation [kWh/m2]: " + Utils::value2String(rad.m_beam, 3));
  m_diffuseRadLabel->setText("Diffuse radiation [kWh/m2]: " + Utils::value2String(rad.m_diffuse, 3));
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


  static bool getSingleDayMode()
  {
    auto iter = m_controls.find("SingleDayMode");
    return ((ParamControlBase*)iter->second)->m_check->getValue();
  }

  static float getStartDay()
  {
    auto iter = m_controls.find("StartDay");
    return ((ParamControlBase*)iter->second)->m_slider->getValue();
  }

  static float getEndDay()
  {
    auto iter = m_controls.find("EndDay");
    return ((ParamControlBase*)iter->second)->m_slider->getValue();
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
      m_parametersControl->setVisible(value);
    }
    else if (m_name == "ToggleResults")
    {
      m_resultLabelsControl->setVisible(value);
    }
    else if (m_name == "ToggleFisheye")
    {
      m_fisheyeControl->setVisible(value);
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
    osg::Vec4 borderColor(0.8, 0.8, 0.8, 1);
    int borderWidth = 2;
    m_namesLabel = new CustomControls::LabelControl("");
    m_namesLabel->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_namesLabel->setVertAlign(CustomControls::Control::ALIGN_CENTER);
    m_namesLabel->setTextBackdropOffset(3);
    m_namesLabel->setFontSize(UI_FONT_SIZE);
    m_namesLabel->setBorderColor(borderColor);
    m_namesLabel->setBorderWidth(borderWidth);

    m_valuesLabel = new CustomControls::LabelControl("");
    m_valuesLabel->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_valuesLabel->setVertAlign(CustomControls::Control::ALIGN_CENTER);
    m_valuesLabel->setTextBackdropOffset(3);
    m_valuesLabel->setFontSize(UI_FONT_SIZE);
    m_valuesLabel->setBorderColor(borderColor);
    m_valuesLabel->setBorderWidth(borderWidth);

    m_fisheyeImagel = new CustomControls::ImageControl;
    m_fisheyeImagel->setSize(256, 256);
    m_fisheyeImagel->setBorderColor(borderColor);
    m_fisheyeImagel->setBorderWidth(borderWidth);

    addControl(m_namesLabel);
    addControl(m_valuesLabel);
    addControl(m_fisheyeImagel);
  }

  void SetPoint(SolarRadiationPoint& point)
  {
    m_point = point;
    std::string names;
    std::string values;
    m_point.toString(names, values);
    m_namesLabel->setText(names);
    m_valuesLabel->setText(values);
    osg::Image* fisheye = m_solarInteractiveHandler->getFisheyeForPoint(m_point.m_id);
    if (fisheye)
      m_fisheyeImagel->setImage(fisheye);
  }

  void setPositionSmart(int x, int y, int viewWidth, int viewHeight)
  {
    if (m_point.m_id < 0)
    {
      setPosition(x, y);
      return;
    }
    float width = renderSize().x();
    float height = renderSize().y();
    int xdiff = (x + width) - viewWidth;
    int ydiff = (y + height) - viewHeight;
    if (xdiff < 0 && ydiff < 0)
    {
      setPosition(x, y);
      return;
    }

    int xoffset = 0;
    if (xdiff > 0 && (x - width) > 0)
    {
      xoffset = -width;
    }

    int yoffset = 0;
    if (ydiff > 0 && (y - height) > 0)
    {
      yoffset = -height;
    }

    setPosition(x + xoffset, y + yoffset);
  }

  SolarRadiationPoint m_point;
  CustomControls::LabelControl* m_namesLabel;
  CustomControls::LabelControl* m_valuesLabel;
  CustomControls::ImageControl* m_fisheyeImagel;
};

void createMainUIControls(CustomControls::ControlCanvas* cs, int viewWidth, int viewHeight)
{
  m_mainUIControl = new CustomControls::VBox();
  m_mainUIControl->setPosition(0, 0);
  m_mainUIControl->setPadding(5);
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
  CustomControls::ImageControl* fishEyeImg = new CustomControls::ImageControl(m_solarInteractiveHandler->fisheyeSurface()->Texture());
  fishEyeImg->setSize(320, 320);
  m_fisheyeControl->addControl(fishEyeImg);
  m_mainUIControl->addControl(togglesControl);
  m_mainUIControl->addControl(m_parametersControl);
  m_mainUIControl->addControl(m_resultLabelsControl);
  m_mainUIControl->addControl(m_fisheyeControl);

  cs->addControl(m_mainUIControl);

  m_compass = createCompass(cs, viewWidth, viewHeight);
  cs->addControl(m_compass);

  m_statusBar = new CustomControls::LabelControl("", fontColor);
  m_statusBar->setFontSize(UI_FONT_SIZE);
  m_statusBar->setBackColor(backgroundColor);
  m_statusBar->setPosition(viewWidth * 0.5 - 50, viewHeight - 50);
  //m_statusBar->setBorderColor(borderColor);
  m_statusBar->setPadding(15);
  cs->addControl(m_statusBar);
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
  popup->m_namesLabel->setFontSize(UI_FONT_SIZE);
  popup->m_namesLabel->setForeColor(fontColor);
  popup->setVisible(false);
  m_popupControl = popup;
  cs->addControl(popup);
}

class MainUIEventHandler : public osgGA::GUIEventHandler
{
private:
  float calAngle(float x, float y)
  {
    float x2 = 0.0;
    float y2 = 1.0;
    float dot = x * x2 + y * y2;      //# dot product\n"
    float det = x * y2 - y * x2;      //# determinant\n"
    float angle = atan2(det, dot) * 57.2958;  //# atan2(y, x) or atan2(sin, cos)\n"
    if (angle < 0)
      angle += 360;
    return angle;
  }
public:
  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
  {
    osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
    if (!viewer)
      return false;

    if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
    {
      if (m_frameCount % 2 == 0)
      {
        float rotateAngle = 0;
        if (m_mapNode)
        {
          osg::Vec3d eye, center, up;
          viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
          osg::Vec3d look = (center - eye);
          look.normalize();
          osg::Vec3d north(0, 0, 1);
          double projectedX = look * north;
          double projectedY = look * osg::Vec3d(1,0,0);
          osg::Vec2d xy(projectedY, projectedX);
          xy.normalize();
          rotateAngle = calAngle(xy.x(), xy.y());
        }
        else 
        {
          osg::Vec3d eye, center, up;
          viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
          osg::Vec3d look = (center - eye);
          look.normalize();
          osg::Vec2d xy(look.x(), look.y());
          xy.normalize();
          rotateAngle = calAngle(xy.x(), xy.y());
        }
        m_compass->getOrCreateStateSet()->getUniform("rotateAngle")->set(rotateAngle);
      }
      return false;
    }

    if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
    {
      osg::Vec3d worldPos, geoPos;
      std::tie(worldPos, geoPos) = m_solarInteractiveHandler->queryCoordinatesAtMouse(ea.getXnormalized(), ea.getYnormalized());
      std::stringstream ss;
      ss.precision(4);

      if (m_mapNode)
      {
        std::string lonFix = geoPos.x() > 0 ? "E" : "W";
        std::string latFix = geoPos.y() > 0 ? "N" : "S";
        std::string eleFix = "m";
        ss << std::fixed << abs(geoPos.x()) << lonFix << " " << abs(geoPos.y()) << latFix << "    " << geoPos.z() << eleFix;
      }
      else
      {
        ss << std::fixed << worldPos.x() << "X" << " " << worldPos.y() << "Y" << " " << worldPos.z() << "Z";
      }
      m_statusBar->setText(ss.str());
    }

    if (ea.getEventType() == osgGA::GUIEventAdapter::RESIZE)
    {
      int viewWidth = viewer->getCamera()->getViewport()->width();
      int viewHeight = viewer->getCamera()->getViewport()->height();
      m_compass->setPosition(viewWidth - 128 - 10, 10);
      m_statusBar->setPosition(viewWidth * 0.5 - 100, viewHeight - 50);
      return false;
    }

    if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK)
      return false;
    if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
      return false;

    int key = ea.getUnmodifiedKey();

    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && key == osgGA::GUIEventAdapter::KEY_H)
    {
      m_mainUIControl->setVisible(!m_mainUIControl->visible());
      return true;
    }

    PopupControl* popup = (PopupControl*)m_popupControl;


    bool handlePointQuery = false;
    if (
      (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP || 
      ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
   && (ea.getUnmodifiedKey() == osgGA::GUIEventAdapter::KEY_Control_L || 
     ea.getUnmodifiedKey() == osgGA::GUIEventAdapter::KEY_Control_R)
      )
    {
      handlePointQuery = true;
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE
      && (ea.getModKeyMask() & ea.MODKEY_CTRL))
    {
      handlePointQuery = true;
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
    {
      popup->setVisible(false);
    }

    if (!handlePointQuery)
    {
      return false;
    }

    SolarRadiationPoint point;
    if (m_solarInteractiveHandler->queryPoint(ea.getXnormalized(), ea.getYnormalized(), point))
    {
      float viewWidth = viewer->getCamera()->getViewport()->width();
      float viewHeight = viewer->getCamera()->getViewport()->height();
      int x = (int)((ea.getXnormalized() * 0.5 + 0.5) * viewWidth) + 10;
      int y = (int)((1.0 - (ea.getYnormalized() * 0.5 + 0.5)) * viewHeight) + 10;
      bool isInitialized = popup->m_point.m_id > -1;
      if (popup->m_point.m_id != point.m_id)
      {
        popup->setPositionSmart(x, y, viewWidth, viewHeight);
        popup->SetPoint(point);
      }
      popup->setVisible(true);
    }
    else
    {
      popup->setVisible(false);
    }

    return false;
  }
};

#include <osg/ComputeBoundsVisitor>
osg::BoundingBoxd calBound(std::string indir, const std::vector<std::string>& tiles)
{
  osg::BoundingBoxd bb;
  bb.init();
  for (size_t i = 0; i < tiles.size(); i++)
  {
    std::string masterTilePath = indir + tiles[i] + "/" + tiles[i] + ".osgb";
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(masterTilePath);
    osg::ComputeBoundsVisitor cbs;
    node->accept(cbs);
    osg::BoundingBoxd tileBB = cbs.getBoundingBox();
    bb.expandBy(tileBB);
  }
  return bb;
}

double calPercentageShaded(const std::string& shadowMasks)
{
  int total = 0;
  int totalShaded = 0;
  for (size_t i = 0; i < shadowMasks.size(); i++)
  {
    char c = shadowMasks[i];
    if (c == ',')
      continue;
    int mask = atoi(&c);
    total++;
    if (mask == 1)
    {
      totalShaded++;
    }
  }
  return (double)totalShaded / (double)total;
}

int main(int argc, char** argv)
{
  m_solarParam = createSolarParam();

  osg::ArgumentParser arguments(&argc, argv);

  //if (argc < 2)
  //  return 0;

  // create a viewer:
  osgViewer::Viewer viewer(arguments);

  // Tell the database pager to not modify the unref settings
  viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, false);

  // thread-safe initialization of the OSG wrapper manager. Calling this here
  // prevents the "unsupported wrapper" messages from OSG
  osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

  std::vector<std::string> tilenames;
  tilenames.push_back("Tile_-002_-016");
  tilenames.push_back("Tile_-002_-017");
  tilenames.push_back("Tile_-003_-016");
  tilenames.push_back("Tile_-003_-017");
  osg::BoundingBoxd bound = calBound("E:/Data/weihai/Data/", tilenames);
  osg::ref_ptr<osg::Node> scene = ModelLoader::Load3DTiles("E:/Data/weihai/Data/", tilenames, false);
  osg::ref_ptr<osg::Node> studyArea = osgDB::readNodeFile("E:/Code/StudyArea.osgb");
  ((osg::Group*)scene.get())->addChild(studyArea.get());
  scene->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
  studyArea->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

  //osg::ref_ptr<osg::Node> scene = MapNodeHelper().load(arguments, &viewer);
  //if (!scene)
  //{
  //  scene = osgDB::readNodeFiles(arguments);
  //}

  //if (!scene)
  //{
  //  scene = ModelLoader::Load3DTiles(arguments[1]);
  //}

  if (!scene)
    return 0;

  osg::ref_ptr<osg::Group> root = new osg::Group;
  root->addChild(scene.get());

  viewer.setSceneData(root.get());

  m_mapNode = MapNode::findMapNode(scene);

  osg::ref_ptr<osgGA::CameraManipulator> manip;
  if (m_mapNode)
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

  m_solarInteractiveHandler = new SolarInteractiveHandler(scene, root, m_mapNode, manip, &viewer, &m_solarParam, onResultsUpdated);


  unsigned int width, height;
  osg::GraphicsContext::ScreenIdentifier main_screen_id;
  osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
  main_screen_id.readDISPLAY();
  main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
  wsi->getScreenResolution(main_screen_id, width, height);

  // create a surface to house the controls
  CustomControls::ControlCanvas* canvas = CustomControls::ControlCanvas::getOrCreate(&viewer);
  // create some controls.
  createMainUIControls(canvas, width, height);
  createPopup(canvas);

  //osg::Group* group = (osg::Group*)viewer.getCamera()->getChild(0);
 // group->addChild(m_northArrow);
  // zoom to a good startup position

  viewer.addEventHandler(m_solarInteractiveHandler);
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
  viewer.setUpViewInWindow(50, 50, 1024, 768);
  viewer.realize();
  for (size_t i = 0; i < 20; i++)
  {
    viewer.frame();
  }
  manip->setHomePosition(bound.center() + osg::Vec3d(0, 0, 5000), bound.center(), osg::Vec3d(0, 1, 0));
  manip->home(0.0);

  ShadowCaster dsmShadowCaster;
  std::vector<std::string> rasterFiles;
  std::vector<osg::BoundingBoxd> bounds;
  rasterFiles.push_back("E:/Code/Weihai_DSM_025_StudyArea.tif");
  //rasterFiles.push_back("E:/Code/Weihai_DSM_1m.tif");
  bounds.push_back(bound);
  //bounds.push_back(osg::BoundingBoxd());
  dsmShadowCaster.Initialize(rasterFiles, bounds, "E:/Code/WeihaiStudyArea/Slope.tif", "E:/Code/WeihaiStudyArea/Aspect.tif");
  GDAL_DS<float>* ds = (GDAL_DS<float>*)dsmShadowCaster.m_rasters[0];

  std::ofstream ofs_inclined("e:/compare_inclined_025.csv");
  std::ofstream ofs_horizontal("e:/compare_horizontal_025.csv");
  ofs_inclined << "pos,row,col,slope2d,slope3d,aspect2d,aspect3d,global2d,global3d,beam2d,beam3d,dif2d,dif3d,ref2d,ref3d,shadow2d,shadow3d\n";
  ofs_horizontal << "pos,row,col,slope2d,slope3d,aspect2d,aspect3d,global2d,global3d,beam2d,beam3d,dif2d,dif3d,ref2d,ref3d,shadow2d,shadow3d\n";
  int sampleCount = 0;
  srand(time(NULL));
  while (sampleCount < 100)
  {
    SolarParam param;
    param.m_linke = 3.0;
    param.m_time_step = 0.5;
    param.m_lon = 122.1204;
    param.m_lat = 37.5131;
    param.m_slope = 0;
    param.m_aspect = 0;
    param.m_isSingleDay = false;
    param.m_day = 183;
    param.m_startDay = 1;
    param.m_endDay = 365;
    double elevatedHeight = 0.1;

    int col, row, index;
    double xrand = rand() / (double)RAND_MAX;
    double yrand = rand() / (double)RAND_MAX;
    double x = bound.xMin() + (bound.xMax() - bound.xMin()) * xrand;
    double y = bound.yMin() + (bound.yMax() - bound.yMin()) * yrand;
    std::tie(col, row, index) = ds->getCellIndex(x, y);
    int margin = 350;
    if (col < margin || row < margin || col > ds->ncols - margin || row > ds->nrows - margin)
      continue;

    osg::Vec3d end = osg::Vec3d(x, y, -1000);
    osg::Vec3d	start = osg::Vec3d(x, y, 1000);
    Ray ray(start, end - start);
    double intersectDist, slope2d, aspect2d, slope3d, aspect3d;
    bool intersects = dsmShadowCaster.Intersects(ray, intersectDist, slope2d, aspect2d);
    osg::Vec3d intersectPos2d = ray.orig + ray.dir * intersectDist;
    intersectPos2d = intersectPos2d + osg::Vec3d(0, 0, elevatedHeight);
    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor intersectVisitor(intersector.get());
    scene->accept(intersectVisitor);
    if (!intersector->containsIntersections())
      continue;

    SolarRadiationPoint inclined2d, horizontal2d, inclined3d, horizontal3d;
    std::tie(inclined2d, horizontal2d) = dsmShadowCaster.calculateSolarRadiation(param, intersectPos2d);
    double percentageShaded = calPercentageShaded(inclined2d.m_shadowMasks);
    if (percentageShaded > 0.75)
      continue;

    osg::Vec3d intersectPos3d = intersector->getFirstIntersection().getWorldIntersectPoint();
    osg::Vec3d intersectNormal = intersector->getFirstIntersection().getWorldIntersectNormal();
    intersectNormal.normalize();
    slope3d = Utils::calculateSlope(intersectNormal);
    aspect3d = Utils::calculateAspect(intersectNormal);
    param.m_slope = slope3d;
    param.m_aspect = aspect3d;
    std::tie(inclined3d, horizontal3d) = m_solarInteractiveHandler->calculateSolarRadiation(&param, intersector, elevatedHeight);
    percentageShaded = calPercentageShaded(inclined3d.m_shadowMasks);
    if (percentageShaded > 0.75)
      continue;

    std::vector<OuputVariable> outputVariables;
    outputVariables.push_back(OuputVariable(intersectPos2d));
    outputVariables.push_back(OuputVariable(row));
    outputVariables.push_back(OuputVariable(col));
    outputVariables.push_back(OuputVariable(slope2d));
    outputVariables.push_back(OuputVariable(slope3d));
    outputVariables.push_back(OuputVariable(aspect2d));
    outputVariables.push_back(OuputVariable(aspect3d));
    outputVariables.push_back(OuputVariable(inclined2d.m_global));
    outputVariables.push_back(OuputVariable(inclined3d.m_global));
    outputVariables.push_back(OuputVariable(inclined2d.m_beam));
    outputVariables.push_back(OuputVariable(inclined3d.m_beam));
    outputVariables.push_back(OuputVariable(inclined2d.m_diffuse));
    outputVariables.push_back(OuputVariable(inclined3d.m_diffuse));
    outputVariables.push_back(OuputVariable(inclined2d.m_reflected));
    outputVariables.push_back(OuputVariable(inclined3d.m_reflected));
    outputVariables.push_back(OuputVariable(inclined2d.m_shadowMasks));
    outputVariables.push_back(OuputVariable(inclined3d.m_shadowMasks));
    for (int v = 0; v < outputVariables.size(); v++)
    {
      outputVariables[v].out(ofs_inclined);
      if (v != outputVariables.size())
      {
        ofs_inclined << ",";
      }
    }
    ofs_inclined << "\n";


    outputVariables.clear();
    outputVariables.push_back(OuputVariable(intersectPos2d));
    outputVariables.push_back(OuputVariable(row));
    outputVariables.push_back(OuputVariable(col));
    outputVariables.push_back(OuputVariable(slope2d));
    outputVariables.push_back(OuputVariable(slope3d));
    outputVariables.push_back(OuputVariable(aspect2d));
    outputVariables.push_back(OuputVariable(aspect3d));
    outputVariables.push_back(OuputVariable(horizontal2d.m_global));
    outputVariables.push_back(OuputVariable(horizontal3d.m_global));
    outputVariables.push_back(OuputVariable(horizontal2d.m_beam));
    outputVariables.push_back(OuputVariable(horizontal3d.m_beam));
    outputVariables.push_back(OuputVariable(horizontal2d.m_diffuse));
    outputVariables.push_back(OuputVariable(horizontal3d.m_diffuse));
    outputVariables.push_back(OuputVariable(horizontal2d.m_reflected));
    outputVariables.push_back(OuputVariable(horizontal3d.m_reflected));
    outputVariables.push_back(OuputVariable(horizontal2d.m_shadowMasks));
    outputVariables.push_back(OuputVariable(horizontal3d.m_shadowMasks));
    for (int v = 0; v < outputVariables.size(); v++)
    {
      outputVariables[v].out(ofs_horizontal);
      if (v != outputVariables.size())
      {
        ofs_horizontal << ",";
      }
    }
    ofs_horizontal << "\n";

    printf("%d/%d\n", sampleCount, 100);
    sampleCount++;
  }
  ofs_inclined.close();
  ofs_horizontal.close();
  return 0;

  while (!viewer.done())
  {
    viewer.frame();
    if (m_frameCount % 10 == 0)
    {
      m_solarInteractiveHandler->postDrawUpdate();
    }
    m_frameCount++;
    if (m_frameCount > 1000000)
      m_frameCount = 1;
  }
  return 0;
}

