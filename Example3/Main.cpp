
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif
#include "SVFComputeTools.h"
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
#include <osgViewer/Viewer>
#include <osgEarth/Registry>
#include <osgEarthUtil/EarthManipulator>
//#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>
#include "CustomControls.h"
//#include "GrassSolar.h"

struct SolarTime
{
  int hour;
  int minute;
  int second;
  SolarTime()
  {
    hour = 6;
    minute = 0;
    second = 0;
  }
  SolarTime(int h, int m, int s)
    :hour(h), minute(m), second(s)
  {
  }
  double toDecimalHour()
  {
    return hour + minute / 60.0 + second / 3600.0;
  }

};
//parameters for r.sun calculation
struct SolarParam
{
  float linke;//turbidity factor
  float bsky;//scale factor for the beam component
  float dsky;//scale factor for the diffuse component
  float lon;//longitude
  float lat;//latitude
  float elev;//elevation
  float slope;//slope in degrees
  float aspect;//aspect in degrees
  float time_step;//time resolution in hours
  int day;//range from 1 to 366
  bool* shadowInfo;//an array of shadow masks corresponding to the number of solar vectors in a day
  bool isShadowed;//a single shadow mask will be used if 'shadowInfo' is null
  bool isInstantaneous;//apply instantaneous calculation mode
  bool isSingleDay;
  bool useLatitudeOverride;
  bool useElevationOverride;
  int startDay;
  int endDay;
  SolarTime time;//decimal time 
  SolarParam()
  {
    shadowInfo = NULL;
    isShadowed = false;
    isInstantaneous = false;//time-integrated calculation mode as default
    elev = 0;
    slope = 0;
    aspect = 0;
    linke = 3.0;
    bsky = 1;
    dsky = 1;
    time_step = 1;
    day = 1;
    startDay = 1;
    endDay = 1;
    isSingleDay = true;
    useLatitudeOverride = true;
    useElevationOverride = true;
  }

};
using namespace osgEarth::Symbology;
//using namespace osgEarth::Util::Controls;
using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

//using namespace CustomControls;
SolarParam m_solarParam;

SolarParam createSolarParam()
{
  SolarParam param;
  param.aspect = 270;
  param.slope = 0;
  param.lon = 122.1204;
  param.lat = 37.5131;
  param.day = 183;
  param.time_step = 0.5;
  param.linke = 3.0;
  param.day = 183;
  return param;
}

void createControls(CustomControls::ControlCanvas*);
CustomControls::ImageControl* s_imageControl = 0L;

struct MyClickHandler : public CustomControls::ControlEventHandler
{
  void onClick(CustomControls::Control* control, const osg::Vec2f& pos, int mouseButtonMask)
  {
    OE_NOTICE << "You clicked at (" << pos.x() << ", " << pos.y() << ") within the control."
      << std::endl;
  }
};

static CustomControls::LabelControl* s_sliderLabel;

struct MySliderHandler : public CustomControls::ControlEventHandler
{
  void onValueChanged(CustomControls::Control* control, float value)
  {

  }

  void onValueChanged(CustomControls::Control* control, bool value)
  {

  }
};

struct RotateImage : public CustomControls::ControlEventHandler
{
  void onValueChanged(CustomControls::Control* control, float value)
  {
    if (s_imageControl)
      s_imageControl->setRotation(Angular(value));
  }
};

void OnSolarParamUpdated(std::string name, float value)
{

}

std::map<std::string, CustomControls::Control*> m_controls;
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
  osg::ref_ptr<CustomControls::HSliderControl> m_slider;
  osg::ref_ptr<CustomControls::CheckBoxControl> m_check;
  std::string m_name;
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

class ParamControl : public ParamControlBase, public CustomControls::ControlEventHandler
{
public:
  ParamControl(std::string name, std::string label, float min, float max, float value, bool isInteger)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    m_isInteger = isInteger;

    setChildSpacing(10);
    setChildVertAlign(CustomControls::Control::ALIGN_CENTER);
    setHorizFill(true);
    setBackColor(osg::Vec4f(0, 0, 1, 0.5));

    osg::ref_ptr <CustomControls::LabelControl> nameLabel = new CustomControls::LabelControl(label);
    nameLabel->setVertAlign(CustomControls::Control::ALIGN_LEFT);
    m_slider = new CustomControls::HSliderControl(min, max, value);
    //slider->setBackColor(.6, 0, 0, 1);
    m_slider->setHeight(25);
    m_slider->setWidth(200);
    m_slider->setName(name);
    m_slider->addEventHandler(this);
    m_valueLabel = new CustomControls::LabelControl(name);
    m_valueLabel->setText(toString(value));
    addControl(nameLabel.get());
    addControl(m_slider.get());
    addControl(m_valueLabel.get());
  }

  ParamControl(std::string name, std::string label, bool isChecked)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    setChildSpacing(10);
    setChildVertAlign(CustomControls::Control::ALIGN_CENTER);
    setHorizFill(true);
    setBackColor(osg::Vec4f(0, 0, 1, 0.5));

    osg::ref_ptr <CustomControls::LabelControl> nameLabel = new CustomControls::LabelControl(label);
    nameLabel->setVertAlign(CustomControls::Control::ALIGN_LEFT);
    m_check = new CustomControls::CheckBoxControl(isChecked);
    //slider->setBackColor(.6, 0, 0, 1);
    m_check->setHeight(25);
    m_check->setName(name);
    m_check->addEventHandler(this);
    addControl(nameLabel.get());
    addControl(m_check.get());
  }

  ParamControl(std::string name, std::string label, float min, float max, float value, bool isInteger, bool isChecked)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    m_isInteger = isInteger;

    setChildSpacing(10);
    setChildVertAlign(CustomControls::Control::ALIGN_CENTER);
    setHorizFill(true);
    setBackColor(osg::Vec4f(0, 0, 1, 0.5));

    osg::ref_ptr <CustomControls::LabelControl> nameLabel = new CustomControls::LabelControl(label);
    nameLabel->setVertAlign(CustomControls::Control::ALIGN_LEFT);
    m_slider = new CustomControls::HSliderControl(min, max, value);
    //slider->setBackColor(.6, 0, 0, 1);
    m_slider->setHeight(25);
    m_slider->setWidth(200);
    m_slider->setName(name);
    m_slider->addEventHandler(this);
    m_valueLabel = new CustomControls::LabelControl(name);
    m_valueLabel->setText(toString(value));
    m_check = new CustomControls::CheckBoxControl(isChecked);
    //slider->setBackColor(.6, 0, 0, 1);
    m_check->setHeight(25);
    m_check->setName(name);
    m_check->addEventHandler(this);
    addControl(nameLabel.get());
    addControl(m_slider.get());
    addControl(m_valueLabel.get());
    addControl(m_check.get());
  }

private:
  osg::ref_ptr<CustomControls::LabelControl> m_valueLabel;
  bool m_isInteger;
  std::string toString(float value)
  {
    //bool isInt = (floor(value) == value);
    std::stringstream buf;
    if (m_isInteger)
    {
      buf << (int)value;
    }
    else
    {
      buf.precision(2);
      buf << std::fixed << value;
    }
    return buf.str();
  }

  void onValueChanged(CustomControls::Control* control, float value)
  {
    if (GetSingleDayMode())
    {
      if (m_name == "StartDay")
      {
        auto iter = m_controls.find("EndDay");
        ParamControl* endSlider = (ParamControl*)iter->second;
        endSlider->m_slider->setValue(value);
        m_solarParam.startDay = (int)value;
      }
      else if (m_name == "EndDay")
      {
        auto iter = m_controls.find("StartDay");
        ParamControl* endSlider = (ParamControl*)iter->second;
        endSlider->m_slider->setValue(value);
        m_solarParam.endDay = (int)value;
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
    }
    if (m_isInteger)
      value = (int)value;
    m_valueLabel->setText(toString(value));
    OnSolarParamUpdated(m_name, value);
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
  }
};

void createControls(CustomControls::ControlCanvas* cs)
{
  osg::ref_ptr<CustomControls::VBox> ul = new CustomControls::VBox();
  ul->setPosition(20, 20);
  ul->setPadding(10);
  osg::ref_ptr <CustomControls::LabelControl> titleLabel = new CustomControls::LabelControl("GRASS GIS r.sun parameters\n(https://grass.osgeo.org/grass78/manuals/r.sun.html)");
  osg::ref_ptr<CustomControls::HBox> linkieSlider = new ParamControl("Linkie", "Linkie", 3, 8, m_solarParam.linke, true);
  osg::ref_ptr<CustomControls::HBox> startDaySlider = new ParamControl("StartDay", "Start Day", 1, 365, m_solarParam.startDay, true);
  osg::ref_ptr<CustomControls::HBox> endDaySlider = new ParamControl("EndDay", "End Day", 1, 365, m_solarParam.endDay, true);
  osg::ref_ptr<CustomControls::HBox> isSingleDayCheck = new ParamControl("SingleDayMode", "Single Day Mode", m_solarParam.isSingleDay);
  osg::ref_ptr<CustomControls::HBox> latSlider = new ParamControl("Latitude", "Latitude Override", -90, 90, m_solarParam.lat,false, true);
  osg::ref_ptr<CustomControls::HBox> elevSlider = new ParamControl("Elevation", "Elevation Override", 0, 10000, m_solarParam.elev, false, true);

  ul->addControl(titleLabel.get());
  ul->addControl(linkieSlider.get());
  ul->addControl(startDaySlider.get());
  ul->addControl(endDaySlider.get());
  ul->addControl(isSingleDayCheck.get());
  ul->addControl(latSlider.get());
  ul->addControl(elevSlider.get());

  cs->addControl(ul.get());
}

#define IMAGERY_URL      "http://readymap.org/readymap/tiles/1.0.0/22/"
#define ELEVATION_URL    "http://readymap.org/readymap/tiles/1.0.0/116/"
#define BUILDINGS_URL    "./data/boston_buildings_utm19.shp"
#define RESOURCE_LIB_URL "./data/resources/textures_us/catalog.xml"
#define STREETS_URL      "./data/boston-scl-utm19n-meters.shp"
#define PARKS_URL        "./data/boston-parks.shp"
#define TREE_MODEL_URL   "./data/tree.osg"

// forward declarations.
void addImagery(Map* map);
void addElevation(Map* map);
void addBuildings(Map* map);
void addStreets(Map* map);
void addParks(Map* map);

osgEarth::ProfileOptions ProfileOptionsFromFile(std::string filename)
{
  osgEarth::ProfileOptions opt;
  if (filename.substr(filename.length() - 4, 4) == ".shp")
  {
    char srs[512];
    memset(srs, 0, 512);
    char* psrs = (char*)srs;
    ShapeFile shp(filename);
    shp.poLayer->GetSpatialRef()->exportToWkt(&psrs);
    opt.srsString() = psrs;
    OGREnvelope env;
    shp.poLayer->GetExtent(&env);
    opt.bounds() = osgEarth::Bounds(env.MinX, env.MinY, env.MaxX, env.MaxY);
  }
  else
  {
    double adfGeoTransform[6];
    GDALDataset* pDataset = (GDALDataset*)GDALOpen(filename.data(), GA_ReadOnly);
    pDataset->GetGeoTransform(adfGeoTransform);
    int ncols = pDataset->GetRasterXSize();
    int nrows = pDataset->GetRasterYSize();
    opt.bounds() = osgEarth::Bounds(adfGeoTransform[0], adfGeoTransform[3] + adfGeoTransform[5] * nrows, adfGeoTransform[3], adfGeoTransform[0] + adfGeoTransform[1] * ncols);
    opt.srsString() = pDataset->GetProjectionRef();
  }
  return opt;
}


void addImagery(Map* map)
{
  // add a TMS imagery layer:
  TMSOptions imagery;
  imagery.url() = IMAGERY_URL;
  map->addLayer(new ImageLayer("ReadyMap imagery", imagery));
}


void addElevation(Map* map)
{
  // add a TMS elevation layer:
  TMSOptions elevation;
  elevation.url() = ELEVATION_URL;
  map->addLayer(new ElevationLayer("ReadyMap elevation", elevation));
}


void addBuildings(Map* map)
{
  // create a feature source to load the building footprint shapefile.
  OGRFeatureOptions buildingData;
  buildingData.name() = "buildings";
  buildingData.url() = BUILDINGS_URL;
  buildingData.buildSpatialIndex() = true;

  // a style for the building data:
  Style buildingStyle;
  buildingStyle.setName("buildings");

  // Extrude the shapes into 3D buildings.
  ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
  extrusion->heightExpression() = NumericExpression("3.5 * max( [story_ht_], 1 )");
  extrusion->flatten() = true;
  extrusion->wallStyleName() = "building-wall";
  extrusion->roofStyleName() = "building-roof";

  PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
  poly->fill()->color() = Color::White;

  // Clamp the buildings to the terrain.
  AltitudeSymbol* alt = buildingStyle.getOrCreate<AltitudeSymbol>();
  alt->clamping() = alt->CLAMP_TO_TERRAIN;
  alt->binding() = alt->BINDING_VERTEX;

  // a style for the wall textures:
  Style wallStyle;
  wallStyle.setName("building-wall");
  SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
  wallSkin->library() = "us_resources";
  wallSkin->addTag("building");
  wallSkin->randomSeed() = 1;

  // a style for the rooftop textures:
  Style roofStyle;
  roofStyle.setName("building-roof");
  SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
  roofSkin->library() = "us_resources";
  roofSkin->addTag("rooftop");
  roofSkin->randomSeed() = 1;
  roofSkin->isTiled() = true;

  // assemble a stylesheet and add our styles to it:
  StyleSheet* styleSheet = new StyleSheet();
  styleSheet->addStyle(buildingStyle);
  styleSheet->addStyle(wallStyle);
  styleSheet->addStyle(roofStyle);

  // load a resource library that contains the building textures.
  ResourceLibrary* reslib = new ResourceLibrary("us_resources", RESOURCE_LIB_URL);
  styleSheet->addResourceLibrary(reslib);

  // set up a paging layout for incremental loading. The tile size factor and
  // the visibility range combine to determine the tile size, such that
  // tile radius = max range / tile size factor.
  FeatureDisplayLayout layout;
  layout.tileSize() = 500;
  layout.addLevel(FeatureLevel(0.0f, 20000.0f, "buildings"));

  FeatureModelLayer* layer = new FeatureModelLayer();
  layer->setName("Buildings");
  layer->options().featureSource() = buildingData;
  layer->options().styles() = styleSheet;
  layer->options().layout() = layout;

  map->addLayer(layer);
}


void addStreets(Map* map)
{
  // create a feature source to load the street shapefile.
  OGRFeatureOptions feature_opt;
  feature_opt.name() = "streets";
  feature_opt.url() = STREETS_URL;
  feature_opt.buildSpatialIndex() = true;

  // a resampling filter will ensure that the length of each segment falls
  // within the specified range. That can be helpful to avoid cropping 
  // very long lines segments.
  ResampleFilterOptions resample;
  resample.minLength() = 0.0f;
  resample.maxLength() = 25.0f;
  feature_opt.filters().push_back(resample);

  // a style:
  Style style;
  style.setName("streets");

  // Render the data as translucent yellow lines that are 7.5m wide.
  LineSymbol* line = style.getOrCreate<LineSymbol>();
  line->stroke()->color() = Color(Color::Yellow, 0.5f);
  line->stroke()->width() = 7.5f;
  line->stroke()->widthUnits() = Units::METERS;

  // Clamp the lines to the terrain.
  AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
  alt->clamping() = alt->CLAMP_TO_TERRAIN;

  // Apply a depth offset to avoid z-fighting. The "min bias" is the minimum
  // apparent offset (towards the camera) of the geometry from its actual position.
  // The value here was chosen empirically by tweaking the "oe_doff_min_bias" uniform.
  RenderSymbol* render = style.getOrCreate<RenderSymbol>();
  render->depthOffset()->minBias() = 6.6f;

  // Set up a paging layout. The tile size factor and the visibility range combine
  // to determine the tile size, such that tile radius = max range / tile size factor.
  FeatureDisplayLayout layout;
  layout.tileSize() = 500;
  layout.maxRange() = 5000.0f;

  // create a model layer that will render the buildings according to our style sheet.
  FeatureModelLayerOptions streets;
  streets.name() = "streets";
  streets.featureSource() = feature_opt;
  streets.layout() = layout;
  streets.styles() = new StyleSheet();
  streets.styles()->addStyle(style);

  map->addLayer(new FeatureModelLayer(streets));
}


void addParks(Map* map)
{
  // create a feature source to load the shapefile.
  OGRFeatureOptions parksData;
  parksData.name() = "parks";
  parksData.url() = PARKS_URL;
  parksData.buildSpatialIndex() = true;

  // a style:
  Style style;
  style.setName("parks");

  // Render the data using point-model substitution, which replaces each point
  // in the feature geometry with an instance of a 3D model. Since the input
  // data are polygons, the PLACEMENT_RANDOM directive below will scatter
  // points within the polygon boundary at the specified density.
  ModelSymbol* model = style.getOrCreate<ModelSymbol>();
  model->url()->setLiteral(TREE_MODEL_URL);
  model->placement() = model->PLACEMENT_RANDOM;
  model->density() = 6000.0f; // instances per sqkm

  // Clamp to the terrain:
  AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
  alt->clamping() = alt->CLAMP_TO_TERRAIN;

  // Since the tree model contains alpha components, we will discard any data
  // that's sufficiently transparent; this will prevent depth-sorting anomalies
  // common when rendering lots of semi-transparent objects.
  RenderSymbol* render = style.getOrCreate<RenderSymbol>();
  render->transparent() = true;
  render->minAlpha() = 0.15f;

  // Set up a paging layout. The tile size factor and the visibility range combine
  // to determine the tile size, such that tile radius = max range / tile size factor.
  FeatureDisplayLayout layout;
  layout.tileSize() = 650;
  layout.addLevel(FeatureLevel(0.0f, 2000.0f, "parks"));

  // create a model layer that will render the buildings according to our style sheet.
  FeatureModelLayerOptions parks;
  parks.name() = "parks";
  parks.featureSource() = parksData;
  parks.layout() = layout;
  parks.styles() = new StyleSheet();
  parks.styles()->addStyle(style);

  Layer* parksLayer = new FeatureModelLayer(parks);
  map->addLayer(parksLayer);

  if (parksLayer->getStatus().isError())
  {
    OE_WARN << parksLayer->getStatus().message() << std::endl;
  }
}

int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc, argv);
  osgViewer::Viewer* viewer = new osgViewer::Viewer(arguments);
  viewer->setUpViewAcrossAllScreens();
  GDALAllRegister();

  m_solarParam = createSolarParam();
  ProfileOptions profileOpt = ProfileOptionsFromFile(PARKS_URL);
  MapOptions mapOpt;
  mapOpt.coordSysType() = MapOptions::CSTYPE_PROJECTED;
  mapOpt.profile() = profileOpt;

  //cache map tiles
  FileSystemCacheOptions cacheOpt;
  cacheOpt.rootPath() = "./cache";
  mapOpt.cache() = cacheOpt;
  mapOpt.cachePolicy() = osgEarth::CachePolicy::USAGE_READ_WRITE;
  Map* map = new Map(mapOpt);

  addImagery(map);
  addElevation(map);
  addStreets(map);
  addBuildings(map);
  addParks(map);

  osg::ref_ptr<EarthManipulator> manip = new EarthManipulator;
  viewer->setCameraManipulator(manip.get());

  osg::ref_ptr<osg::Group> root = new osg::Group();
  viewer->setSceneData(root.get());
  // make the map scene graph:
  MapNode* mapNode = new MapNode(map);
  root->addChild(mapNode);

  // create a surface to house the controls
  CustomControls::ControlCanvas* cs = CustomControls::ControlCanvas::getOrCreate(viewer);

  // create some controls.
  createControls(cs);
  // zoom to a good startup position
  manip->setViewpoint(Viewpoint(
    "Home",
    -71.0763, 42.34425, 0,   // longitude, latitude, altitude
    24.261, -21.6, 3450.0), // heading, pitch, range
    5.0);                    // duration

  viewer->addEventHandler(new SkyViewFactorEventHandler(mapNode, root, manip, viewer));

  viewer->addEventHandler(new osgViewer::ThreadingHandler);

  // add the window size toggle handler
  viewer->addEventHandler(new osgViewer::WindowSizeHandler);

  // add the stats handler
  viewer->addEventHandler(new osgViewer::StatsHandler);

  // add the LOD Scale handler
  viewer->addEventHandler(new osgViewer::LODScaleHandler);
  return viewer->run();
}
