// OS識別用
var os;

//  // DOM構築完了イベントハンドラ登録
window.addEventListener("DOMContentLoaded", init);

 // 初期化
function init() {
    // 簡易的なOS判定
    os = detectOSSimply();
    if (os == "iphone") {
        // safari用。DeviceOrientation APIの使用をユーザに許可して貰う
        document.getElementById("permitOrientation").addEventListener("click", permitDeviceOrientationForSafari);
        document.getElementById("permitMotion").addEventListener("click", permitDeviceMotionForSafari);
        window.addEventListener(
            "deviceorientation",
            handleOrientation,
            true
        );
        window.addEventListener(
            "devicemotion",
            handleMotion,
            true
        );
        
    } else if (os == "android") {
        window.addEventListener(
            "deviceorientationabsolute",
            handleOrientation,
            true
        );
        window.addEventListener(
            "devicemotion",
            handleMotion,
            true
        );
    } else{
        window.alert("PC未対応サンプル");
    }
}

var absolute = 0;
var alpha = 0;
var beta = 0;
var gamma = 0;
var degrees = 0;
  // ジャイロスコープと地磁気をセンサーから取得
function handleOrientation(event) {
    absolute = event.absolute;
    alpha = event.alpha;
    beta = event.beta;
    gamma = event.gamma;


    if(os == "iphone") {
        // webkitCompasssHeading値を採用
        degrees = event.webkitCompassHeading;

     }else{
        // deviceorientationabsoluteイベントのalphaを補正
        degrees = compassHeading(alpha, beta, gamma);
    }

    var direction;
    if (
        (degrees > 337.5 && degrees < 360) ||
        (degrees > 0 && degrees < 22.5)
    ) {
        direction = "北";
    } else if (degrees > 22.5 && degrees < 67.5) {
        direction = "北東";
    } else if (degrees > 67.5 && degrees < 112.5) {
        direction = "東";
    } else if (degrees > 112.5 && degrees < 157.5) {
        direction = "東南";
    } else if (degrees > 157.5 && degrees < 202.5) {
        direction = "南";
    } else if (degrees > 202.5 && degrees < 247.5) {
        direction = "南西";
    } else if (degrees > 247.5 && degrees < 292.5) {
        direction = "西";
    } else if (degrees > 292.5 && degrees < 337.5) {
        direction = "北西";
    }
}

 // 端末の傾き補正（Android用）
// https://www.w3.org/TR/orientation-event/
function compassHeading(alpha, beta, gamma) {
    var degtorad = Math.PI / 180; // Degree-to-Radian conversion

    var _x = beta ? beta * degtorad : 0; // beta value
    var _y = gamma ? gamma * degtorad : 0; // gamma value
    var _z = alpha ? alpha * degtorad : 0; // alpha value

    var cX = Math.cos(_x);
    var cY = Math.cos(_y);
    var cZ = Math.cos(_z);
    var sX = Math.sin(_x);
    var sY = Math.sin(_y);
    var sZ = Math.sin(_z);

     // Calculate Vx and Vy components
    var Vx = -cZ * sY - sZ * sX * cY;
    var Vy = -sZ * sY + cZ * sX * cY;

     // Calculate compass heading
    var compassHeading = Math.atan(Vx / Vy);

     // Convert compass heading to use whole unit circle
    if (Vy < 0) {
        compassHeading += Math.PI;
    } else if (Vx < 0) {
        compassHeading += 2 * Math.PI;
    }

     return compassHeading * (180 / Math.PI); // Compass Heading (in degrees)
}
// 簡易OS判定
function detectOSSimply() {
    let ret;
    if (
        navigator.userAgent.indexOf("iPhone") > 0 ||
        navigator.userAgent.indexOf("iPad") > 0 ||
        navigator.userAgent.indexOf("iPod") > 0
    ) {
        // iPad OS13のsafariはデフォルト「Macintosh」なので別途要対応
        ret = "iphone";
    } else if (navigator.userAgent.indexOf("Android") > 0) {
        ret = "android";
    } else {
        ret = "pc";
    }

    return ret;
}

var accX = 0;
var accY = 0;
var accZ = 0;

  // ジャイロスコープと地磁気をセンサーから取得
function handleMotion(event) {
    accX = event.accelerationIncludingGravity.x;
    accY = event.accelerationIncludingGravity.y;
    accZ = event.accelerationIncludingGravity.z;

    if(os == "iphone") {

    }else{
        
    }
}

// iPhone + Safariの場合はDeviceOrientation APIの使用許可をユーザに求める
function permitDeviceOrientationForSafari() {
    DeviceOrientationEvent.requestPermission()
        .then(response => {
            if (response === "granted") {
                window.addEventListener(
                    "deviceorientation",
                    detectDirection
                );
                document.getElementById("permitOrientation").style.display ="none";
            }
        })
        .catch(console.error);
}

function permitDeviceMotionForSafari() {
    DeviceMotionEvent.requestPermission()
        .then(response => {
            if (response === "granted") {
                window.addEventListener(
                    "devicemotion",
                    detectMotion
                );
                document.getElementById("permitMotion").style.display ="none";
            }
        })
        .catch(console.error);
}



var Id = -1;
var usedId = [];
window.onload = function(){
      setTimeout(() => {
        if(Id < 0){
          Id = 0;
        }
      },2000);
        var Talker = {
          ros : null,
          name : "",
          init : function(){
              this.ros = new ROSLIB.Ros();
              this.ros.on('error', function(error) {
                  document.getElementById('state').innerHTML = "Error";
              });
              this.ros.on('connection', function(error) {
                  document.getElementById('state').innerHTML = "Connect";
              });
              this.ros.on('close', function(error) {
                  document.getElementById('state').innerHTML = "Close";
              });
              this.ros.connect('wss://' + location.hostname + ':9090');
              var sub1 = new ROSLIB.Topic({
                  ros : this.ros,
                  name : '/angleidcheck',
                  messageType : 'std_msgs/Int32'
              });
              sub1.subscribe(function(message){
                usedId.push(message.data);
                if(Id==-1){
                  Id=-2;
                setTimeout(() => {
                  var j = 0;
                  while(Id<0){
                    if(usedId.includes(j)){}
                    else{Id = j};
                    j++;
                  }
                },1000);
              }
              });
              var sub2 = new ROSLIB.Topic({
                  ros : this.ros,
                  name : '/client_count',
                  messageType : 'std_msgs/Int32'
              });
              sub2.subscribe(function(message) {
                usedId = [];
                if(Id >= 0){
                  var pub = new ROSLIB.Topic({
                    ros : this.ros,
                    name : '/angleidcheck',
                    messageType : 'std_msgs/Int32',
                    url: 'wss://' + ip + ':9090'
                  });
                  var idData = new ROSLIB.Message({data : Id});
                  setTimeout(() => {
                    pub.publish(idData);
                  },100+10*Id);
                }
              });
          },
          send : function(msg,ip){
              var pub = new ROSLIB.Topic({
                  ros : this.ros,
                  name : '/imu',
                  messageType : 'sensor_msgs/Imu',
                  url: 'wss://' + ip + ':9090'
              });
              pub.publish(msg);
          },
          getIp : function(){
            if(document.getElementById("IP").value.length == 0) return;
            return document.getElementById("IP").value;
          },
          getAngle : function(ID){
            var angledata = new ROSLIB.Message({
              header : { frame_id : "angle"+ID },
              orientation : {x : gamma,y : beta,z : alpha},
              orientation_covariance : [0,0,0,0,0,0,0,0,0],
              angular_velocity : {x : 0,y : 0,z : 0},
              angular_velocity_covariance : [0,0,0,0,0,0,0,0,0],
              linear_acceleration : {x : accX,y : accY,z : accZ},
              linear_acceleration_covariance : [0,0,0,0,0,0,0,0,0],
            });
            return angledata;
          }
        }
      Talker.init();
    //   init();
      
      setInterval(function (){
        angle_data = Talker.getAngle(Id);
        ip = Talker.getIp();
        //console.log(ip);
        if(Id>=0){
          Talker.send(angle_data,ip);
        }
      },100);
      };
      window.onunload = function(){
          Talker.ros.close();
      };
      window.oncontextmenu = function(event) {
       event.preventDefault();
       event.stopPropagation();
       return false;
    };


