console.log('connecting to websocket...')
var socket = io('ws://' + networkValues['address'] +':5800/ws');
console.log('connected to websocket!')

var pipelines = null;
var activePipeline = null;
var results = null;

var defaultPipelineVals = {'name': 'Pipeline', 'streamType': 'result', 'ledMode': 0, 'ledBrightness': 0, 'shutterSpeed': 2, 'awbGains': [0.0, 0.0], 'iso': 400, 'flipImg': false, 'blurOption': 'none', 'aperatureSize': 3, 'gaussianKernelSize': [3, 3], "sigmaX": 1, "sigmaY": 1, "hsvLow": [0, 0, 0], "hsvHigh": [180, 255, 255], "performErosion": false, "shapeOfErosionKernel": "rectangle", "sizeOfErosionKernel": [3, 3], "erosionKernelAnchor": [-1, -1], "erosionAnchor": [-1, -1], "erosionIterations": 1, "performDilation": false, "shapeOfDilationKernel": "rectangle", "sizeOfDilationKernel": [3, 3], "dilationKernelAnchor": [-1, -1], "dilationAnchor": [-1, -1], "dilationIterations": 1, "useConvexHull": false, "contourAreaRange": [0, 1], "contourFullnessRange": [0, 1], "contourAspectRatioRange": [0, 10], "contourSortingMode": "center", "contourConvexHull": false, "numOfContourCorners": 0, "pairContours": false, "contourIntersectionLocation": "neither", "targetAreaRange": [0, 1], "targetFullnessRange": [0, 1], "targetAspectRatioRange": [0, 10], "targetSortingMode": "center", "targetConvexHull": false, "numOfPairCorners": 0, "performPoseEstimation": false, "poseEstimationRange": null, "objectPoints": null, "targetName": null};

// Initial Setup Stuff
document.getElementById("PipelineConfiguratorTabButton").click();
document.getElementById("CameraSetupTabButton").click();

function withinRange(val, min, max) {
  if (val < min || val > max || isNaN(val)) {
    return false;
  } else {
    return true;
  }
}

// Tab Changing Stuff
function changeTab(evt, tabName) {
  // Declare all variables
  var i, tabcontent, tablinks;

  // Get all elements with class="tabcontent" and hide them
  tabcontent = document.getElementsByClassName("tabcontent");
  for (i = 0; i < tabcontent.length; i++) {
    tabcontent[i].style.display = "none";
  }

  // Get all elements with class="tablinks" and remove the class "active"
  tablinks = document.getElementsByClassName("tablinks");
  for (i = 0; i < tablinks.length; i++) {
    tablinks[i].className = tablinks[i].className.replace(" active", "");
  }

  // Show the current tab, and add an "active" class to the button that opened the tab
  document.getElementById(tabName).style.display = "block";
  evt.currentTarget.className += " active";
}

function changePipelineStage(evt, stage) {
  // Declare all variables
  var i, tabcontent, tablinks;

  // Get all elements with class="tabcontent" and hide them
  tabcontent = document.getElementsByClassName("_tabcontent");
  for (i = 0; i < tabcontent.length; i++) {
    tabcontent[i].style.display = "none";
  }

  // Get all elements with class="tablinks" and remove the class "active"
  tablinks = document.getElementsByClassName("_tablinks");
  for (i = 0; i < tablinks.length; i++) {
    tablinks[i].className = tablinks[i].className.replace(" active", "");
  }

  // Show the current tab, and add an "active" class to the button that opened the tab
  if (stage == "ThresholdingTab") {
    pipelines[activePipeline]["streamType"] = "thresh"
  } else {
    pipelines[activePipeline]["streamType"] = "results"
  }
  document.getElementById(stage).style.display = "block";
  evt.currentTarget.className += " active";
}

// Various setup methods

function setupTopBar(pipeline) {
  document.getElementById("PipelineChooser").value = pipeline;
  pipelineChooserCallback()
  document.getElementById("ActivePipelineNameEntry").value = pipelines[pipeline]['name'];
}

function setupCameraSetupDiv(pipeline) {
  // Get the info we need from the pipeline
  shutterSpeed = pipelines[pipeline]["shutterSpeed"];
  awbGains = pipelines[pipeline]["awbGains"];
  iso = pipelines[pipeline]["iso"];
  flipImg = pipelines[pipeline]["flipImg"];
  enableLEDs = pipelines[pipeline]["ledMode"];

  // Update the GUI
  shutterSpeedCallback(shutterSpeed.toString());
  redBalanceCallback(awbGains[0].toString());
  blueBalanceCallback(awbGains[1].toString());
  isoCallback(iso.toString());
  document.getElementById("FlipImgCheckbox").checked = flipImg;
  if (enableLEDs == 1) {
    document.getElementById("EnableLEDsCheckbox").checked = true;
  } else {
    document.getElementById("EnableLEDsCheckbox").checked = false;
  }
}

function setupThresholdingDiv(pipeline) {
  // Get our info from the pipline
  hsvLow = pipelines[pipeline]["hsvLow"];
  hsvHigh = pipelines[pipeline]["hsvHigh"];
  
  // Update the GUI
  lowHueCallback(hsvLow[0].toString());
  lowSaturationCallback(hsvLow[1].toString());
  lowValueCallback(hsvLow[2].toString());
  highHueCallback(hsvHigh[0].toString());
  highSaturationCallback(hsvHigh[1].toString());
  highValueCallback(hsvHigh[2].toString());
}

function setupErosion(pipeline) {
  // Get our info from the pipeline
  enableErosion = pipelines[pipeline]["performErosion"];
  erosionIterations = pipelines[pipeline]["erosionIterations"];
  erosionAnchor = pipelines[pipeline]["erosionAnchor"];
  erosionKernelShape = pipelines[pipeline]["shapeOfErosionKernel"];
  erosionKernelSize = pipelines[pipeline]["sizeOfErosionKernel"];
  erosionKernelAnchor = pipelines[pipeline]["erosionKernelAnchor"];

  // Update the GUI
  // Setup the enable erosion checkbox
  document.getElementById("EnableErosionCheckbox").checked = enableErosion;

  // Setup the erosion iterations checkbox
  erosionIterationsCallback(erosionIterations.toString());

  // Setup the erosion kernel size and the anchors
  erosionKernelWidthCallback(erosionKernelSize[0], erosionAnchor[0], erosionKernelAnchor[0]);
  erosionKernelHeightCallback(erosionKernelSize[1], erosionAnchor[1], erosionKernelAnchor[1]);

  // Setup the erosion kernel shape
  document.getElementById("ErosionKernelShape").value = erosionKernelShape;
}

function setupContourFiltering(pipeline) {
  // First grab all the data from the pipeline
  contourAreaRange = pipelines[pipeline]["contourAreaRange"];
  contourFullnessRange = pipelines[pipeline]["contourFullnessRange"];
  contourAspectRatioRange = pipelines[pipeline]["contourAspectRatioRange"];
  contourSortingMode = pipelines[pipeline]["contourSortingMode"];

  // Now, update the GUI
  lowContourAreaCallback(contourAreaRange[0]);
  highContourAreaCallback(contourAreaRange[1]);
  lowContourFullnessCallback(contourFullnessRange[0]);
  highContourFullnessCallback(contourFullnessRange[1]);
  lowContourAspectRatioCallback(contourAspectRatioRange[0]);
  highContourAspectRatioCallback(contourAspectRatioRange[1]);

  document.getElementById("ContourSortingMode").value = contourSortingMode;
}

function setupDilation(pipeline) {
  // Get our info from the pipeline
  enableDilation = pipelines[pipeline]["performDilation"];
  dilationIterations = pipelines[pipeline]["dilationIterations"];
  dilationAnchor = pipelines[pipeline]["dilationAnchor"];
  dilationKernelShape = pipelines[pipeline]["shapeOfDilationKernel"];
  dilationKernelSize = pipelines[pipeline]["sizeOfDilationKernel"];
  dilationKernelAnchor = pipelines[pipeline]["dilationKernelAnchor"];

  // Update the GUI
  // Setup the enable dilation checkbox
  document.getElementById("EnableDilationCheckbox").checked = enableDilation;

  // Setup the dilation iterations checkbox
  dilationIterationsCallback(dilationIterations.toString());

  // Setup the dilation kernel size and the anchors
  dilationKernelWidthCallback(dilationKernelSize[0], dilationAnchor[0], dilationKernelAnchor[0]);
  dilationKernelHeightCallback(dilationKernelSize[1], dilationAnchor[1], dilationKernelAnchor[1]);

  // Setup the dilation kernel shape
  document.getElementById("DilationKernelShape").value = dilationKernelShape;
}

function setupContourFiltering(pipeline) {
  // First grab all the data from the pipeline
  contourAreaRange = pipelines[pipeline]["contourAreaRange"];
  contourFullnessRange = pipelines[pipeline]["contourFullnessRange"];
  contourAspectRatioRange = pipelines[pipeline]["contourAspectRatioRange"];
  contourConvexHull = pipelines[pipeline]["contourConvexHull"];
  contourCorners = pipelines[pipeline]["numOfContourCorners"];
  contourSortingMode = pipelines[pipeline]["contourSortingMode"];

  // Now, update the GUI
  lowContourAreaCallback(contourAreaRange[0]);
  highContourAreaCallback(contourAreaRange[1]);
  lowContourFullnessCallback(contourFullnessRange[0]);
  highContourFullnessCallback(contourFullnessRange[1]);
  lowContourAspectRatioCallback(contourAspectRatioRange[0]);
  highContourAspectRatioCallback(contourAspectRatioRange[1]);
  document.getElementById("ContourConvexHullCheckbox").checked = contourConvexHull;
  contourCornersCallback(contourCorners);

  document.getElementById("ContourSortingMode").value = contourSortingMode;
}

function setupTargetFiltering(pipeline) {
  // First grab all the data from the pipeline
  pairContours = pipelines[pipeline]["pairContours"];
  contourIntersectionLocation = pipelines[pipeline]["contourIntersectionLocation"];
  targetAreaRange = pipelines[pipeline]["targetAreaRange"];
  targetFullnessRange = pipelines[pipeline]["targetFullnessRange"];
  targetAspectRatioRange = pipelines[pipeline]["targetAspectRatioRange"];
  targetConvexHull = pipelines[pipeline]["targetConvexHull"];
  targetCorners = pipelines[pipeline]["numOfPairCorners"]
  targetSortingMode = pipelines[pipeline]["targetSortingMode"];

  // Now, update the GUI
  document.getElementById("EnableContourPairingCheckbox").checked = pairContours;
  document.getElementById("ContourIntersectionLocation").value = contourIntersectionLocation;

  lowTargetAreaCallback(targetAreaRange[0]);
  highTargetAreaCallback(targetAreaRange[1]);
  lowTargetFullnessCallback(targetFullnessRange[0]);
  highTargetFullnessCallback(targetFullnessRange[1]);
  lowTargetAspectRatioCallback(targetAspectRatioRange[0]);
  highTargetAspectRatioCallback(targetAspectRatioRange[1]);
  document.getElementById("TargetConvexHullCheckbox").checked = targetConvexHull;
  targetCornersCallback(targetCorners);

  document.getElementById("TargetSortingMode").value = targetSortingMode;
}

function setupPoseEstimation(pipeline) {
  // First grab all the data we need form the pipeline
  performPoseEstimation = pipelines[pipeline]["performPoseEstimation"];
  targetName = pipelines[pipeline]["targetName"];
  points = pipelines[pipeline]["objectPoints"];
  range = pipelines[pipeline]["poseEstimationRange"];

  // Now, update the GUI
  document.getElementById("EnablePoseEstimationCheckbox").checked = performPoseEstimation;
  if (targetName != null) {
    document.getElementById("TargetModelName").textContent = targetName;
  } else {
    document.getElementById("TargetModelName").textContent = "";
  }

  if (performPoseEstimation) {
    fromScale = document.getElementById("FromScale");
    fromScale.max = points.length;
    fromCallback(1);
    toScale = document.getElementById("ToScale");
    toScale.max = points.length;
    toCallback(points.length);
  } else {
    fromScale = document.getElementById("FromScale");
    fromScale.max = 1;
    fromCallback(1);
    toScale = document.getElementById("ToScale");
    toScale.max = 1;
    toCallback(1);
  }
}

// Various Handlers for UI elements

// Handlers for top bar
const uploadPipelineElement = document.getElementById("uploadPipeline");
uploadPipelineElement.addEventListener("change", uploadPipelineCallback, false);
function uploadPipelineCallback() {
  const fileList = this.files;
  reader = new FileReader();
  fileList[0].text().then(text => {
  	pipelines[activePipeline] = JSON.parse(text);
  }, errorInfo => {
  	console.log("WE DUN GOOFED");
  });
  uploadPipelineElement.value = null;
}

function pipelineChooserCallback() {
  activePipeline = parseInt(document.getElementById("PipelineChooser").value);
  document.getElementById("ActivePipelineNameEntry").value = pipelines[activePipeline]['name'];
  setupCameraSetupDiv(activePipeline);
  setupThresholdingDiv(activePipeline);
  setupErosion(activePipeline)
  setupDilation(activePipeline)
  setupContourFiltering(activePipeline);
  setupTargetFiltering(activePipeline);
  setupPoseEstimation(activePipeline);
}

function activePipelineNameEntryCallback(value) {
  pipelines[activePipeline]['name'] = value;
  
  var pipelineSelect = document.getElementById('PipelineChooser');
  // pipelineSelect.remove(pipelineSelect.selectedIndex);
  pipelineSelect.options[activePipeline].text = value;
  
  // var newOption = document.createElement('option');
  // newOption.appendChild(document.createTextNode(newName));
  // newOption.value = newName;
  // document.getElementById("PipelineChooser").appendChild(newOption);

  // document.getElementById("PipelineChooser").value = newName;
}

// Handlers for camera setup tab
function shutterSpeedCallback(value) {
  var newShutterSpeed = Number(value);
  if (!withinRange(newShutterSpeed, 2, 20)) {
    newShutterSpeed = pipelines[activePipeline]['shutterSpeed'];
  }
  document.getElementById("ShutterSpeedEntry").value = newShutterSpeed.toString();
  document.getElementById("ShutterSpeedScale").value = newShutterSpeed;
  pipelines[activePipeline]['shutterSpeed'] = newShutterSpeed;
}

function redBalanceCallback(value) {
  var newRedBalance = Number(value);
  if (!withinRange(newRedBalance, 0.0, 8.0)) {
    newRedBalance = pipelines[activePipeline]['awbGains'][0];
  }
  document.getElementById("RedBalanceEntry").value = newRedBalance.toString();
  document.getElementById("RedBalanceScale").value = newRedBalance;
  pipelines[activePipeline]['awbGains'][0] = newRedBalance;
}

function blueBalanceCallback(value) {
  var newBlueBalance = Number(value);
  if (!withinRange(newBlueBalance, 0.0, 8.0)) {
    newBlueBalance = pipelines[activePipeline]['awbGains'][1];
  }
  document.getElementById("BlueBalanceEntry").value = newBlueBalance.toString();
  document.getElementById("BlueBalanceScale").value = newBlueBalance;
  pipelines[activePipeline]['awbGains'][1] = newBlueBalance;
}

function isoCallback(value) {
  var newISO = Number(value);
  if (!withinRange(newISO, 0, 1600)) {
    newISO = pipelines[activePipeline]['iso'];
  }
  document.getElementById("ISOEntry").value = newISO.toString();
  document.getElementById("ISOScale").value = newISO;
  pipelines[activePipeline]['iso'] = newISO;
}

function flipImgCheckboxCallback() {
  var flipImg = document.getElementById("FlipImgCheckbox").checked;
  pipelines[activePipeline]["flipImg"] = flipImg;
}

function enableLEDsCheckboxCallback() {
  var enableLEDs = document.getElementById("EnableLEDsCheckbox").checked;
  if (enableLEDs) {
    pipelines[activePipeline]["ledMode"] = 1;
  } else {
    pipelines[activePipeline]["ledMode"] = 0;
  }
}

// Handlers for thresholding tab
function lowHueCallback(value) {
  var newLowHue = Number(value);
  if (!withinRange(newLowHue, 0, 180)) {
    newLowHue = pipelines[activePipeline]['hsvLow'][0];
  }
  document.getElementById("LowHueEntry").value = newLowHue.toString();
  document.getElementById("LowHueScale").value = newLowHue;
  pipelines[activePipeline]['hsvLow'][0] = newLowHue;
}

function lowSaturationCallback(value) {
  var newLowSaturation = Number(value);
  if (!withinRange(newLowSaturation, 0, 255)) {
    newLowSaturation = pipelines[activePipeline]['hsvLow'][1];
  }
  document.getElementById("LowSaturationEntry").value = newLowSaturation.toString();
  document.getElementById("LowSaturationScale").value = newLowSaturation;
  pipelines[activePipeline]['hsvLow'][1] = newLowSaturation;
}

function lowValueCallback(value) {
  var newLowValue = Number(value);
  if (!withinRange(newLowValue, 0, 255)) {
    newLowValue = pipelines[activePipeline]['hsvLow'][2];
  }
  document.getElementById("LowValueEntry").value = newLowValue.toString();
  document.getElementById("LowValueScale").value = newLowValue;
  pipelines[activePipeline]['hsvLow'][2] = newLowValue;
}

function highHueCallback(value) {
  var newHighHue = Number(value);
  if (!withinRange(newHighHue, 0, 180)) {
    newHighHue = pipelines[activePipeline]['hsvHigh'][0];
  }
  document.getElementById("HighHueEntry").value = newHighHue.toString();
  document.getElementById("HighHueScale").value = newHighHue;
  pipelines[activePipeline]['hsvHigh'][0] = newHighHue;
}

function highSaturationCallback(value) {
  var newHighSaturation = Number(value);
  if (!withinRange(newHighSaturation, 0, 255)) {
    newHighSaturation = pipelines[activePipeline]['hsvHigh'][1];
  }
  document.getElementById("HighSaturationEntry").value = newHighSaturation.toString();
  document.getElementById("HighSaturationScale").value = newHighSaturation;
  pipelines[activePipeline]['hsvHigh'][1] = newHighSaturation;
}

function highValueCallback(value) {
  var newHighValue = Number(value);
  if (!withinRange(newHighValue, 0, 255)) {
    newHighValue = pipelines[activePipeline]['hsvHigh'][2];
  }
  document.getElementById("HighValueEntry").value = newHighValue.toString();
  document.getElementById("HighValueScale").value = newHighValue;
  pipelines[activePipeline]['hsvHigh'][2] = newHighValue;
}

// Erosion Callbacks

function enableErosionCheckboxCallback() {
  var enableErosion = document.getElementById("EnableErosionCheckbox").checked;
  pipelines[activePipeline]["performErosion"] = enableErosion;
}

function erosionIterationsCallback(value) {
  var newErosionIterations = Number(value);
  if (!withinRange(newErosionIterations, 0, 5)) {
    newErosionIterations = pipelines[activePipeline]['erosionIterations'];
  }
  document.getElementById("ErosionIterationsEntry").value = newErosionIterations.toString();
  document.getElementById("ErosionIterationsScale").value = newErosionIterations;
  pipelines[activePipeline]['erosionIterations'] = newErosionIterations;
}

function erosionAnchorXCallback(value) {
  var erosionKernelSize = pipelines[activePipeline]["sizeOfErosionKernel"];
  var newErosionAnchorX = Number(value);
  if (!withinRange(newErosionAnchorX, -erosionKernelSize[0], erosionKernelSize[0])) {
    newErosionAnchorX = pipelines[activePipeline]['erosionAnchor'][0];
  }
  document.getElementById("ErosionAnchorXEntry").value = newErosionAnchorX.toString();
  document.getElementById("ErosionAnchorXScale").value = newErosionAnchorX;
  pipelines[activePipeline]['erosionAnchor'][0] = newErosionAnchorX;
}

function erosionAnchorYCallback(value) {
  var erosionKernelSize = pipelines[activePipeline]["sizeOfErosionKernel"];
  var newErosionAnchorY = Number(value);
  if (!withinRange(newErosionAnchorY, -erosionKernelSize[1], erosionKernelSize[1])) {
    newErosionAnchorY = pipelines[activePipeline]['erosionAnchor'][1];
  }
  document.getElementById("ErosionAnchorYEntry").value = newErosionAnchorY.toString();
  document.getElementById("ErosionAnchorYScale").value = newErosionAnchorY;
  pipelines[activePipeline]['erosionAnchor'][1] = newErosionAnchorY;
}

function erosionKernelShapeCallback() {
  var erosionKernelShape = document.getElementById("ErosionKernelShape").value;
  pipelines[activePipeline]['shapeOfErosionKernel'] = erosionKernelShape;
}

function erosionKernelWidthCallback(value, erosionAnchorX, erosionKernelAnchorX) {
  var newErosionKernelWidth = Number(value);
  if (!withinRange(newErosionKernelWidth, 0, 11) || newErosionKernelWidth % 2 == 0) {
    newErosionKernelWidth = pipelines[activePipeline]['sizeOfErosionKernel'][0];
  }
  document.getElementById("ErosionKernelWidthEntry").value = newErosionKernelWidth.toString();
  document.getElementById("ErosionKernelWidthScale").value = newErosionKernelWidth;
  pipelines[activePipeline]['sizeOfErosionKernel'][0] = newErosionKernelWidth;
  
  erosionAnchorXScale = document.getElementById("ErosionAnchorXScale");
  erosionAnchorXScale.max = newErosionKernelWidth;
  erosionAnchorXScale.min = -newErosionKernelWidth;
  if (isNaN(erosionAnchorX)) {
    erosionAnchorXCallback(-1);
  } else {
    erosionAnchorXCallback(erosionAnchorX);
  }

  erosionKernelAnchorXScale = document.getElementById("ErosionKernelAnchorXScale");
  erosionKernelAnchorXScale.max = newErosionKernelWidth;
  erosionKernelAnchorXScale.min = -newErosionKernelWidth;
  if (isNaN(erosionKernelAnchorX)) {
    erosionKernelAnchorXCallback(-1);
  } else {
    erosionKernelAnchorXCallback(erosionKernelAnchorX);
  }
}

function erosionKernelHeightCallback(value, erosionAnchorY, erosionKernelAnchorY) {
  var newErosionKernelHeight = Number(value);
  if (!withinRange(newErosionKernelHeight, 0, 11) || newErosionKernelHeight % 2 == 0) {
    newErosionKernelHeight = pipelines[activePipeline]['sizeOfErosionKernel'][0];
  }
  document.getElementById("ErosionKernelHeightEntry").value = newErosionKernelHeight.toString();
  document.getElementById("ErosionKernelHeightScale").value = newErosionKernelHeight;
  pipelines[activePipeline]['sizeOfErosionKernel'][1] = newErosionKernelHeight;

  erosionAnchorYScale = document.getElementById("ErosionAnchorYScale");
  erosionAnchorYScale.max = newErosionKernelHeight;
  erosionAnchorYScale.min = -newErosionKernelHeight;
  if (isNaN(erosionAnchorY)) {
    erosionAnchorYCallback(-1);
  } else {
    erosionAnchorYCallback(erosionAnchorY);
  }

  erosionKernelAnchorYScale = document.getElementById("ErosionKernelAnchorYScale");
  erosionKernelAnchorYCallback(-1);
  erosionKernelAnchorYScale.max = newErosionKernelHeight;
  erosionKernelAnchorYScale.min = -newErosionKernelHeight;
  if (isNaN(erosionKernelAnchorY)) {
    erosionKernelAnchorYCallback(-1);
  } else {
    erosionKernelAnchorYCallback(erosionKernelAnchorY);
  }
}

function erosionKernelAnchorXCallback(value) {
  var erosionKernelSize = pipelines[activePipeline]["sizeOfErosionKernel"];
  var newErosionKernelAnchorX = Number(value);
  if (!withinRange(newErosionKernelAnchorX, -erosionKernelSize[0], erosionKernelSize[0])) {
    newErosionKernelAnchorX = pipelines[activePipeline]['erosionKernelAnchor'][0];
  }
  document.getElementById("ErosionKernelAnchorXEntry").value = newErosionKernelAnchorX.toString();
  document.getElementById("ErosionKernelAnchorXScale").value = newErosionKernelAnchorX;
  pipelines[activePipeline]['erosionKernelAnchor'][0] = newErosionKernelAnchorX;
}

function erosionKernelAnchorYCallback(value) {
  var erosionKernelSize = pipelines[activePipeline]["sizeOfErosionKernel"];
  var newErosionKernelAnchorY = Number(value);
  if (!withinRange(newErosionKernelAnchorY, -erosionKernelSize[1], erosionKernelSize[1])) {
    newErosionKernelAnchorY = pipelines[activePipeline]['erosionKernelAnchor'][1];
  }
  document.getElementById("ErosionKernelAnchorYEntry").value = newErosionKernelAnchorY.toString();
  document.getElementById("ErosionKernelAnchorYScale").value = newErosionKernelAnchorY;
  pipelines[activePipeline]['erosionKernelAnchor'][1] = newErosionKernelAnchorY;
}

// Dilation Callbacks

function enableDilationCheckboxCallback() {
  var enableDilation = document.getElementById("EnableDilationCheckbox").checked;
  pipelines[activePipeline]["performDilation"] = enableDilation;
}

function dilationIterationsCallback(value) {
  var newDilationIterations = Number(value);
  if (!withinRange(newDilationIterations, 0, 5)) {
    newDilationIterations = pipelines[activePipeline]['dilationIterations'];
  }
  document.getElementById("DilationIterationsEntry").value = newDilationIterations.toString();
  document.getElementById("DilationIterationsScale").value = newDilationIterations;
  pipelines[activePipeline]['dilationIterations'] = newDilationIterations;
}

function dilationAnchorXCallback(value) {
  var dilationKernelSize = pipelines[activePipeline]["sizeOfDilationKernel"];
  var newDilationAnchorX = Number(value);
  if (!withinRange(newDilationAnchorX, -dilationKernelSize[0], dilationKernelSize[0])) {
    newDilationAnchorX = pipelines[activePipeline]['dilationAnchor'][0];
  }
  document.getElementById("DilationAnchorXEntry").value = newDilationAnchorX.toString();
  document.getElementById("DilationAnchorXScale").value = newDilationAnchorX;
  pipelines[activePipeline]['dilationAnchor'][0] = newDilationAnchorX;
}

function dilationAnchorYCallback(value) {
  var dilationKernelSize = pipelines[activePipeline]["sizeOfDilationKernel"];
  var newDilationAnchorY = Number(value);
  if (!withinRange(newDilationAnchorY, -dilationKernelSize[1], dilationKernelSize[1])) {
    newDilationAnchorY = pipelines[activePipeline]['dilationAnchor'][1];
  }
  document.getElementById("DilationAnchorYEntry").value = newDilationAnchorY.toString();
  document.getElementById("DilationAnchorYScale").value = newDilationAnchorY;
  pipelines[activePipeline]['dilationAnchor'][1] = newDilationAnchorY;
}

function dilationKernelShapeCallback() {
  var dilationKernelShape = document.getElementById("DilationKernelShape").value;
  pipelines[activePipeline]['shapeOfDilationKernel'] = dilationKernelShape;
}

function dilationKernelWidthCallback(value, dilationAnchorX, dilationKernelAnchorX) {
  var newDilationKernelWidth = Number(value);
  if (!withinRange(newDilationKernelWidth, 0, 11) || newDilationKernelWidth % 2 == 0) {
    newDilationKernelWidth = pipelines[activePipeline]['sizeOfDilationKernel'][0];
  }
  document.getElementById("DilationKernelWidthEntry").value = newDilationKernelWidth.toString();
  document.getElementById("DilationKernelWidthScale").value = newDilationKernelWidth;
  pipelines[activePipeline]['sizeOfDilationKernel'][0] = newDilationKernelWidth;
  
  dilationAnchorXScale = document.getElementById("DilationAnchorXScale");
  dilationAnchorXScale.max = newDilationKernelWidth;
  dilationAnchorXScale.min = -newDilationKernelWidth;
  if (isNaN(dilationAnchorX)) {
    dilationAnchorXCallback(-1);
  } else {
    dilationAnchorXCallback(dilationAnchorX);
  }

  dilationKernelAnchorXScale = document.getElementById("DilationKernelAnchorXScale");
  dilationKernelAnchorXScale.max = newDilationKernelWidth;
  dilationKernelAnchorXScale.min = -newDilationKernelWidth;
  if (isNaN(dilationKernelAnchorX)) {
    dilationKernelAnchorXCallback(-1);
  } else {
    dilationKernelAnchorXCallback(dilationKernelAnchorX);
  }
}

function dilationKernelHeightCallback(value, dilationAnchorY, dilationKernelAnchorY) {
  var newDilationKernelHeight = Number(value);
  if (!withinRange(newDilationKernelHeight, 0, 11) || newDilationKernelHeight % 2 == 0) {
    newDilationKernelHeight = pipelines[activePipeline]['sizeOfDilationKernel'][0];
  }
  document.getElementById("DilationKernelHeightEntry").value = newDilationKernelHeight.toString();
  document.getElementById("DilationKernelHeightScale").value = newDilationKernelHeight;
  pipelines[activePipeline]['sizeOfDilationKernel'][1] = newDilationKernelHeight;

  dilationAnchorYScale = document.getElementById("DilationAnchorYScale");
  dilationAnchorYScale.max = newDilationKernelHeight;
  dilationAnchorYScale.min = -newDilationKernelHeight;
  if (isNaN(dilationAnchorY)) {
    dilationAnchorYCallback(-1);
  } else {
    dilationAnchorYCallback(dilationAnchorY);
  }

  dilationKernelAnchorYScale = document.getElementById("DilationKernelAnchorYScale");
  dilationKernelAnchorYCallback(-1);
  dilationKernelAnchorYScale.max = newDilationKernelHeight;
  dilationKernelAnchorYScale.min = -newDilationKernelHeight;
  if (isNaN(dilationKernelAnchorY)) {
    dilationKernelAnchorYCallback(-1);
  } else {
    dilationKernelAnchorYCallback(dilationKernelAnchorY);
  }
}

function dilationKernelAnchorXCallback(value) {
  var dilationKernelSize = pipelines[activePipeline]["sizeOfDilationKernel"];
  var newDilationKernelAnchorX = Number(value);
  if (!withinRange(newDilationKernelAnchorX, -dilationKernelSize[0], dilationKernelSize[0])) {
    newDilationKernelAnchorX = pipelines[activePipeline]['dilationKernelAnchor'][0];
  }
  document.getElementById("DilationKernelAnchorXEntry").value = newDilationKernelAnchorX.toString();
  document.getElementById("DilationKernelAnchorXScale").value = newDilationKernelAnchorX;
  pipelines[activePipeline]['dilationKernelAnchor'][0] = newDilationKernelAnchorX;
}

function dilationKernelAnchorYCallback(value) {
  var DilationKernelSize = pipelines[activePipeline]["sizeOfDilationKernel"];
  var newDilationKernelAnchorY = Number(value);
  if (!withinRange(newDilationKernelAnchorY, -dilationKernelSize[1], dilationKernelSize[1])) {
    newDilationKernelAnchorY = pipelines[activePipeline]['dilationKernelAnchor'][1];
  }
  document.getElementById("DilationKernelAnchorYEntry").value = newDilationKernelAnchorY.toString();
  document.getElementById("DilationKernelAnchorYScale").value = newDilationKernelAnchorY;
  pipelines[activePipeline]['dilationKernelAnchor'][1] = newDilationKernelAnchorY;
}

// Handlers for the contour filtering page
function lowContourAreaCallback(value) {
  var newLowContourArea = Number(value);
  if (!withinRange(newLowContourArea, 0.0, 1.0)) {
    newLowContourArea = pipelines[activePipeline]['contourAreaRange'][0];
  }
  document.getElementById("LowContourAreaEntry").value = newLowContourArea.toString();
  document.getElementById("LowContourAreaScale").value = newLowContourArea;
  pipelines[activePipeline]['contourAreaRange'][0] = newLowContourArea;
}

function highContourAreaCallback(value) {
  var newHighContourArea = Number(value);
  if (!withinRange(newHighContourArea, 0.0, 1.0)) {
    newHighContourArea = pipelines[activePipeline]['contourAreaRange'][1];
  }
  document.getElementById("HighContourAreaEntry").value = newHighContourArea.toString();
  document.getElementById("HighContourAreaScale").value = newHighContourArea;
  pipelines[activePipeline]['contourAreaRange'][1] = newHighContourArea;
}

function lowContourFullnessCallback(value) {
  var newLowContourFullness = Number(value);
  if (!withinRange(newLowContourFullness, 0.0, 1.0)) {
    newLowContourFullness = pipelines[activePipeline]['contourFullnessRange'][0];
  }
  document.getElementById("LowContourFullnessEntry").value = newLowContourFullness.toString();
  document.getElementById("LowContourFullnessScale").value = newLowContourFullness;
  pipelines[activePipeline]['contourFullnessRange'][0] = newLowContourFullness;
}

function highContourFullnessCallback(value) {
  var newHighContourFullness = Number(value);
  if (!withinRange(newHighContourFullness, 0.0, 1.0)) {
    newHighContourFullness = pipelines[activePipeline]['contourFullnessRange'][1];
  }
  document.getElementById("HighContourFullnessEntry").value = newHighContourFullness.toString();
  document.getElementById("HighContourFullnessScale").value = newHighContourFullness;
  pipelines[activePipeline]['contourFullnessRange'][1] = newHighContourFullness;
}

function lowContourAspectRatioCallback(value) {
  var newLowContourAspectRatio = Number(value);
  if (!withinRange(newLowContourAspectRatio, 0.0, 10.0)) {
    newLowContourAspectRatio = pipelines[activePipeline]['contourAspectRatioRange'][0];
  }
  document.getElementById("LowContourAspectRatioEntry").value = newLowContourAspectRatio.toString();
  document.getElementById("LowContourAspectRatioScale").value = newLowContourAspectRatio;
  pipelines[activePipeline]['contourAspectRatioRange'][0] = newLowContourAspectRatio;
}

function highContourAspectRatioCallback(value) {
  var newHighContourAspectRatio = Number(value);
  if (!withinRange(newHighContourAspectRatio, 0.0, 10.0)) {
    newHighContourAspectRatio = pipelines[activePipeline]['contourAspectRatioRange'][1];
  }
  document.getElementById("HighContourAspectRatioEntry").value = newHighContourAspectRatio.toString();
  document.getElementById("HighContourAspectRatioScale").value = newHighContourAspectRatio;
  pipelines[activePipeline]['contourAspectRatioRange'][1] = newHighContourAspectRatio;
}

function contourConvexHullCheckboxCallback() {
  var contourConvexHull = document.getElementById("ContourConvexHullCheckbox").checked;
  pipelines[activePipeline]["contourConvexHull"] = contourConvexHull;
}

function contourCornersCallback(value) {
  var newContourCorners = Number(value);
  if (!withinRange(newContourCorners, 0.0, 10.0)) {
    newContourCorners = pipelines[activePipeline]['numOfContourCorners'];
  }
  document.getElementById("ContourCornersEntry").value = newContourCorners.toString();
  pipelines[activePipeline]['numOfContourCorners'] = newContourCorners;
}

function contourSortingModeCallback() {
  var sortingMode = document.getElementById("ContourSortingMode").value;
  pipelines[activePipeline]["contourSortingMode"] = sortingMode;
}

// Handlers for target filtering page
function enableContourPairingCheckboxCallback() {
  var enableContourPairing = document.getElementById("EnableContourPairingCheckbox").checked;
  pipelines[activePipeline]["pairContours"] = enableContourPairing;
}

function contourIntersectionLocationCallback() {
  var contourIntersectionLocation = document.getElementById("ContourIntersectionLocation").value;
  pipelines[activePipeline]["contourIntersectionLocation"] = contourIntersectionLocation;
}

function lowTargetAreaCallback(value) {
  var newLowTargetArea = Number(value);
  if (!withinRange(newLowTargetArea, 0.0, 1.0)) {
    newLowTargetArea = pipelines[activePipeline]['targetAreaRange'][0];
  }
  document.getElementById("LowTargetAreaEntry").value = newLowTargetArea.toString();
  document.getElementById("LowTargetAreaScale").value = newLowTargetArea;
  pipelines[activePipeline]['targetAreaRange'][0] = newLowTargetArea;
}

function highTargetAreaCallback(value) {
  var newHighTargetArea = Number(value);
  if (!withinRange(newHighTargetArea, 0.0, 1.0)) {
    newHighTargetArea = pipelines[activePipeline]['targetAreaRange'][1];
  }
  document.getElementById("HighTargetAreaEntry").value = newHighTargetArea.toString();
  document.getElementById("HighTargetAreaScale").value = newHighTargetArea;
  pipelines[activePipeline]['targetAreaRange'][1] = newHighTargetArea;
}

function lowTargetFullnessCallback(value) {
  var newLowTargetFullness = Number(value);
  if (!withinRange(newLowTargetFullness, 0.0, 1.0)) {
    newLowTargetFullness = pipelines[activePipeline]['targetFullnessRange'][0];
  }
  document.getElementById("LowTargetFullnessEntry").value = newLowTargetFullness.toString();
  document.getElementById("LowTargetFullnessScale").value = newLowTargetFullness;
  pipelines[activePipeline]['targetFullnessRange'][0] = newLowTargetFullness;
}

function highTargetFullnessCallback(value) {
  var newHighTargetFullness = Number(value);
  if (!withinRange(newHighTargetFullness, 0.0, 1.0)) {
    newHighTargetFullness = pipelines[activePipeline]['targetFullnessRange'][1];
  }
  document.getElementById("HighTargetFullnessEntry").value = newHighTargetFullness.toString();
  document.getElementById("HighTargetFullnessScale").value = newHighTargetFullness;
  pipelines[activePipeline]['targetFullnessRange'][1] = newHighTargetFullness;
}

function lowTargetAspectRatioCallback(value) {
  var newLowTargetAspectRatio = Number(value);
  if (!withinRange(newLowTargetAspectRatio, 0.0, 10.0)) {
    newLowTargetAspectRatio = pipelines[activePipeline]['targetAspectRatioRange'][0];
  }
  document.getElementById("LowTargetAspectRatioEntry").value = newLowTargetAspectRatio.toString();
  document.getElementById("LowTargetAspectRatioScale").value = newLowTargetAspectRatio;
  pipelines[activePipeline]['targetAspectRatioRange'][0] = newLowTargetAspectRatio;
}

function highTargetAspectRatioCallback(value) {
  var newHighTargetAspectRatio = Number(value);
  if (!withinRange(newHighTargetAspectRatio, 0.0, 10.0)) {
    newHighTargetAspectRatio = pipelines[activePipeline]['targetAspectRatioRange'][1];
  }
  document.getElementById("HighTargetAspectRatioEntry").value = newHighTargetAspectRatio.toString();
  document.getElementById("HighTargetAspectRatioScale").value = newHighTargetAspectRatio;
  pipelines[activePipeline]['targetAspectRatioRange'][1] = newHighTargetAspectRatio;
}

function targetConvexHullCheckboxCallback() {
  var targetConvexHull = document.getElementById("TargetConvexHullCheckbox").checked;
  pipelines[activePipeline]["targetConvexHull"] = targetConvexHull;
}

function targetCornersCallback(value) {
  var newTargetCorners = Number(value);
  if (!withinRange(newTargetCorners, 0, 20)) {
    newTargetCorners = pipelines[activePipeline]['numOfPairCorners'];
  }
  document.getElementById("TargetCornersEntry").value = newTargetCorners.toString();
  pipelines[activePipeline]['numOfPairCorners'] = newTargetCorners;
}

function targetSortingModeCallback() {
  var sortingMode = document.getElementById("TargetSortingMode").value;
  pipelines[activePipeline]["targetSortingMode"] = sortingMode;
}

// Handlers for Pose Estimation page
function enablePoseEstimationCheckboxCallback() {
  performPoseEstimation = document.getElementById("EnablePoseEstimationCheckbox").checked;
  if (performPoseEstimation) {
    if (pipelines[activePipeline]["objectPoints"] == null) {
      document.getElementById("EnablePoseEstimationCheckbox").checked = false;
      alert("Upload a target model first!");
    } else {
      pipelines[activePipeline]["performPoseEstimation"] = performPoseEstimation;
      fromScale = document.getElementById("FromScale");
      fromScale.max = pipelines[activePipeline]['objectPoints'].length;
      fromCallback(1);
      toScale = document.getElementById("ToScale");
      toScale.max = pipelines[activePipeline]['objectPoints'].length;
      toCallback(pipelines[activePipeline]['objectPoints'].length);
    }
  } else {
    pipelines[activePipeline]["performPoseEstimation"] = performPoseEstimation;
    fromScale = document.getElementById("FromScale");
    fromScale.max = 1;
    fromCallback(1);
    toScale = document.getElementById("ToScale");
    toScale.max = 1;
    toCallback(1);
  }
}

const uploadTargetModelElement = document.getElementById("uploadTargetModel");
uploadTargetModelElement.addEventListener("change", uploadTargetModelCallback, false);
function uploadTargetModelCallback() {
  const fileList = this.files;
  reader = new FileReader();
  fileList[0].text().then(text => {
    targetModel = JSON.parse(text)
    pipelines[activePipeline]['targetName'] = targetModel['name'];
    pipelines[activePipeline]['objectPoints'] = targetModel['points'];
    pipelines[activePipeline]['poseEstimationRange'] = [1, targetModel['points'].length]
  }, errorInfo => {
  	console.log("WE DUN GOOFED");
  });
  uploadTargetModelElement.value = null;
}

function fromCallback(value) {
  var performPoseEstimation = pipelines[activePipeline]["performPoseEstimation"];
  if (performPoseEstimation) {
    var range = pipelines[activePipeline]["poseEstimationRange"];
    var points = pipelines[activePipeline]["objectPoints"];
    var newFrom = Number(value);
    if (!withinRange(newFrom, 1, points.length) || (range[1] - newFrom + 1) < 4 || newFrom >= range[1]) {
      newFrom = range[0];
    }
    document.getElementById("FromEntry").value = newFrom.toString();
    document.getElementById("FromScale").value = newFrom;
    pipelines[activePipeline]['poseEstimationRange'][0] = newFrom;
  } else {
    document.getElementById("FromEntry").value = "1";
    document.getElementById("FromScale").value = 1;
  }
}

function toCallback(value) {
  var performPoseEstimation = pipelines[activePipeline]["performPoseEstimation"];
  if (performPoseEstimation) {
    var range = pipelines[activePipeline]["poseEstimationRange"];
    var points = pipelines[activePipeline]["objectPoints"];
    var newTo = Number(value);
    if (!withinRange(newTo, 1, points.length) || (newTo - range[0] + 1) < 4 || newTo <= range[0]) {
      newTo = range[1];
    }
    document.getElementById("ToEntry").value = newTo.toString();
    document.getElementById("ToScale").value = newTo;
    pipelines[activePipeline]['poseEstimationRange'][1] = newTo;
  } else {
    document.getElementById("ToEntry").value = "1";
    document.getElementById("ToScale").value = 1;
  }
}

// Handlers for Settings Page
function updateTeamNumber() {
  networkValues['teamNumber'] = document.getElementById('TeamNumberInput').value;
  socket.emit('updateNetworkingValues', JSON.stringify(networkValues));
}

function updateIPAddressAssignmentMode() {
  assignmentMode = document.getElementById('IPAddressAssignmentMode').value;
  if (assignmentMode == 'static') {
    networkValues['static'] = true;
  } else {
    networkValues['static'] = false;
  }
  socket.emit('update_network_values', JSON.stringify(networkValues))
}

function updateIPAddress() {
  networkValues['address'] = document.getElementById('IPAddressInput').value;
  socket.emit('update_network_values', JSON.stringify(networkValues));
}

function updateGateway() {
  networkValues['gateway'] = document.getElementById('GatewayInput').value;
  socket.emit('update_network_values', JSON.stringify(networkValues));
}

function updateNetmask() {
  networkValues['netmask'] = document.getElementById('NetmaskInput').value;
  socket.emit('update_network_values', JSON.stringify(networkValues));
}

function updateRunNetworktables() {
  networkValues['runNetworktables'] = !document.getElementById('DisableNetworkTablesCheckbox').checked;
  socket.emit('update_network_values', JSON.stringify(networkValues))
}

// Websocket Stuff

// event handler for when we connect to the server
socket.on('connect', function() {
  console.log('connected to server!');
});

// event handler for when we recieve a pipeline from ther server (this only happens when we first connect)
socket.on('initData', function(pipelineData) {
  console.log('recieved pipeline!');
  // Parse the data we receive
  pipelineData = JSON.parse(pipelineData)
  activePipeline = pipelineData['activePipeline']
  pipelines = pipelineData['pipelines']

  // Go through the parsed data, create new entries for the dropdown menu in the topbar
  for (var i = 0; i < pipelines.length; i++) {
    var newOption = document.createElement('option');
    newOption.appendChild(document.createTextNode(pipelines[i]['name']));
    newOption.value = i;
    document.getElementById("PipelineChooser").appendChild(newOption);
  }

  setupTopBar(activePipeline)

  socket.emit('updatePipelineVals', JSON.stringify({'activePipeline': activePipeline, 'vals': pipelines[activePipeline]}));
});

// event handler for when a pipeline is uploaded
socket.on('newPipelineVals', function(newPipelineVals) {
  console.log('received new pipeline vals');
  newPipelineVals = JSON.parse(newPipelineVals);
  pipeline[activePipeline] = newPipelineVals;
});

// event handler for when we recieve the results of the pipeline to display in the interface
socket.on('pipelineResults', function(data) {
  console.log('pipeline results sent!');
  results = JSON.parse(data);
  if (results['tv'] == 0) {
    document.getElementById('noTargetLabel').style.visibility = 'visible';
    document.getElementById('txWrapper').style.visibility = 'hidden';
    document.getElementById('tyWrapper').style.visibility = 'hidden';
    document.getElementById('tlWrapper').style.visibility = 'hidden';
    document.getElementById('translationWrapper').style.visibility = 'hidden';
    document.getElementById('rotationWrapper').style.visibility = 'hidden';
  } else {
    document.getElementById('noTargetLabel').style.visibility = 'hidden';
    document.getElementById('txWrapper').style.visibility = 'visible';
    document.getElementById('tyWrapper').style.visibility = 'visible';
    document.getElementById('tlWrapper').style.visibility = 'visible';
    document.getElementById('translationWrapper').style.visibility = 'visible';
    document.getElementById('rotationWrapper').style.visibility = 'visible';
    document.getElementById('tx').textContent = results['tx']
    document.getElementById('ty').textContent = results['ty']
    document.getElementById('tl').textContent = results['tl']
    document.getElementById('translation').textContent = results['translation']
    document.getElementById('rotation').textContent = results['rotation']
  }
  socket.emit('updatePipelineVals', JSON.stringify({'activePipeline': parseInt(activePipeline), 'vals': pipelines[activePipeline]}));
});

// event handler for when we disconnect from the server
socket.on('disconnect', function() {
  console.log('disconnected from server!');
});
