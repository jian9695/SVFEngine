
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
#include "GrassSolar.h"

using namespace osgEarth::Symbology;
using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

SolarParam m_solarParam;
osg::ref_ptr<SkyViewFactorEventHandler> m_skyViewHandler;

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
  param.elev = 500;
  return param;
}

void createControls(CustomControls::ControlCanvas*);
CustomControls::ImageControl* s_imageControl = 0L;

std::map<std::string, CustomControls::Control*> m_controls;
osg::ref_ptr<CustomControls::VBox> m_parametersControl;
osg::ref_ptr<CustomControls::VBox> m_resultsControl;
osg::ref_ptr <CustomControls::LabelControl > m_svfLabel;

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
  osg::ref_ptr <CustomControls::LabelControl> m_nameLabel;
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

std::string PadRight(std::string str, const size_t num, const char paddingChar = ' ')
{
  if (num > str.size())
    str.insert(str.size(), num - str.size(), paddingChar);
  return str;
}

std::string Value2String(float value, int isInteger = false)
{
  //bool isInt = (floor(value) == value);
  std::stringstream buf;
  std::string str = "";
  if (isInteger)
  {
    buf << (int)value;
    str = buf.str();
  }
  else
  {
    buf.precision(2);
    buf << std::fixed << value;
    str = buf.str();
  }
  return str;
}

void ResultsUpdated(float svf, SolarRadiation rad)
{
  m_svfLabel->setText("SVF: " + Value2String(svf));
  //m_svfLabel = new CustomControls::LabelControl("SVF:");
  //osg::ref_ptr <CustomControls::LabelControl> globalRadLabel = new CustomControls::LabelControl("Global radiation:");
  //osg::ref_ptr <CustomControls::LabelControl> beamRadLabel = new CustomControls::LabelControl("Beam radiation:");
  //osg::ref_ptr <CustomControls::LabelControl> DiffuseRadLabel = new CustomControls::LabelControl("Diffuse radiation:");
}

class ParamControl : public ParamControlBase, public CustomControls::ControlEventHandler
{
public:

  void Init(std::string name, std::string label, bool isInteger)
  {
    m_isInteger = isInteger;

    setChildSpacing(10);
    setChildVertAlign(CustomControls::Control::ALIGN_CENTER);
    setHorizFill(true);
    m_nameLabel = new CustomControls::LabelControl(label);
    m_nameLabel->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_nameLabel->setVertAlign(CustomControls::Control::ALIGN_CENTER);
    m_nameLabel->setTextBackdropOffset(3);
    //m_nameLabel->setBackColor(.6, .6, 0, 1);
    //m_nameLabel->setForeColor(0, 0, 0, 1);
    m_nameLabel->setHorizFill(true, 200);
    addControl(m_nameLabel.get());

  }
  ParamControl(std::string name, std::string label, float min, float max, float value, bool isInteger)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    Init(name, label, isInteger);

    m_slider = new CustomControls::HSliderControl(min, max, value);
    //m_slider->setBackColor(.6, 0, 0, 1);
    m_slider->setHeight(25);
    m_slider->setWidth(200);
    m_slider->setName(name);
    m_slider->addEventHandler(this);
    m_slider->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    m_valueLabel = new CustomControls::LabelControl(name);
    m_valueLabel->setText(toString(value));
    addControl(m_slider.get());
    addControl(m_valueLabel.get());
  }

  ParamControl(std::string name, std::string label, bool isChecked)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    Init(name, label, false);

    m_check = new CustomControls::CheckBoxControl(isChecked);
    //slider->setBackColor(.6, 0, 0, 1);
    m_check->setHeight(25);
    m_check->setName(name);
    m_check->addEventHandler(this);
    addControl(m_check.get());
  }

  ParamControl(std::string name, std::string label, float min, float max, float value, bool isInteger, bool isChecked)
    :ParamControlBase(name), CustomControls::ControlEventHandler()
  {
    Init(name, label, isInteger);

    m_slider = new CustomControls::HSliderControl(min, max, value);
    m_slider->setHorizAlign(CustomControls::Control::ALIGN_LEFT);
    //m_slider->setBackColor(.6, 0, 0, 1);
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
      m_resultsControl->setNodeMask(value);
    }
  }
};

void createControls(CustomControls::ControlCanvas* cs)
{
  osg::ref_ptr<CustomControls::VBox> ul = new CustomControls::VBox();
  ul->setPosition(0, 0);
  ul->setPadding(10);
  int maxLabelLen = 20;
  osg::Vec4 backgroundColor(0.0, 0.0, 1.0, 0.3);
  osg::Vec4 borderColor(0.0, 0.0, 0.0, 1.0);

  osg::ref_ptr<CustomControls::VBox> togglesControl = new CustomControls::VBox();
  osg::ref_ptr<CustomControls::HBox> toggleParameters = new ParamControl("ToggleParameters", "Toggle Hide/Show Parameters", true);
  osg::ref_ptr<CustomControls::HBox> toggleResults = new ParamControl("ToggleResults", "Toggle Hide/Show Results", true);
  togglesControl->addControl(toggleParameters.get());
  togglesControl->addControl(toggleResults.get());
  togglesControl->setBackColor(backgroundColor);
  togglesControl->setBorderColor(borderColor);

  m_parametersControl = new CustomControls::VBox();
  m_parametersControl->setBackColor(backgroundColor);
  m_parametersControl->setBorderColor(borderColor);
  osg::ref_ptr <CustomControls::LabelControl> titleLabel = new CustomControls::LabelControl("GRASS GIS r.sun parameters\n(https://grass.osgeo.org/grass78/manuals/r.sun.html)");
  osg::ref_ptr<CustomControls::HBox> linkieSlider = new ParamControl("Linkie", PadRight("Linkie", maxLabelLen), 3, 8, m_solarParam.linke, true);
  osg::ref_ptr<CustomControls::HBox> startDaySlider = new ParamControl("StartDay", PadRight("Start Day", maxLabelLen), 1, 365, m_solarParam.startDay, true);
  osg::ref_ptr<CustomControls::HBox> endDaySlider = new ParamControl("EndDay", PadRight("End Day", maxLabelLen), 1, 365, m_solarParam.endDay, true);
  osg::ref_ptr<CustomControls::HBox> isSingleDayCheck = new ParamControl("SingleDayMode", PadRight("Single Day Mode", maxLabelLen), m_solarParam.isSingleDay);
  osg::ref_ptr<CustomControls::HBox> latSlider = new ParamControl("Latitude", PadRight("Latitude Override", maxLabelLen), -90, 90, m_solarParam.lat, false, true);
  osg::ref_ptr<CustomControls::HBox> elevSlider = new ParamControl("Elevation", PadRight("Elevation Override", maxLabelLen), 0, 9999, m_solarParam.elev, false, true);
  m_parametersControl->addControl(titleLabel.get());
  m_parametersControl->addControl(isSingleDayCheck.get());
  m_parametersControl->addControl(startDaySlider.get());
  m_parametersControl->addControl(endDaySlider.get());
  m_parametersControl->addControl(latSlider.get());
  m_parametersControl->addControl(elevSlider.get());
  m_parametersControl->addControl(linkieSlider.get());

  m_resultsControl = new CustomControls::VBox();
  m_resultsControl->setBackColor(backgroundColor);
  m_resultsControl->setBorderColor(borderColor);
  m_svfLabel = new CustomControls::LabelControl("SVF:");
  osg::ref_ptr <CustomControls::LabelControl> globalRadLabel = new CustomControls::LabelControl("Global radiation:");
  osg::ref_ptr <CustomControls::LabelControl> beamRadLabel = new CustomControls::LabelControl("Beam radiation:");
  osg::ref_ptr <CustomControls::LabelControl> DiffuseRadLabel = new CustomControls::LabelControl("Diffuse radiation:");
  osg::ref_ptr<CustomControls::ImageControl> fishEyeImg = new CustomControls::ImageControl(m_skyViewHandler->_cubemap2fisheyeCamera->Texture());
  m_resultsControl->addControl(m_svfLabel.get());
  m_resultsControl->addControl(globalRadLabel.get());
  m_resultsControl->addControl(beamRadLabel.get());
  m_resultsControl->addControl(DiffuseRadLabel.get());
  m_resultsControl->addControl(fishEyeImg.get());

  ul->addControl(togglesControl.get());
  ul->addControl(m_parametersControl.get());
  ul->addControl(m_resultsControl.get());

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
  manip->setViewpoint(Viewpoint(
    "Home",
    -71.0763, 42.34425, 0,   // longitude, latitude, altitude
    24.261, -21.6, 3450.0), // heading, pitch, range
    5.0);                    // duration
  m_skyViewHandler = new SkyViewFactorEventHandler(mapNode, root, manip, viewer, ResultsUpdated);
  // create a surface to house the controls
  CustomControls::ControlCanvas* cs = CustomControls::ControlCanvas::getOrCreate(viewer);

  // create some controls.
  createControls(cs);
  // zoom to a good startup position

  viewer->addEventHandler(m_skyViewHandler.get());

  viewer->addEventHandler(new osgViewer::ThreadingHandler);

  // add the window size toggle handler
  viewer->addEventHandler(new osgViewer::WindowSizeHandler);

  // add the stats handler
  viewer->addEventHandler(new osgViewer::StatsHandler);

  // add the LOD Scale handler
  viewer->addEventHandler(new osgViewer::LODScaleHandler);
  return viewer->run();
}
