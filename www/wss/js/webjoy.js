class button{
    constructor(id){
      this._id = id;
      this._obj = document.getElementById(id);
      this._value = 0;
      this._obj.addEventListener('mousedown',()=>{this.setValue(1);},false);
      this._obj.addEventListener('touchstart',()=>{this.setValue(1);},false);
      this._obj.addEventListener('mouseup',()=>{this.setValue(0);},false);
      this._obj.addEventListener('touchend',()=>{this.setValue(0);},false);
    }
  
    setValue(v){
      this._value = v;
    }
  
    getValue(){
      return this._value;
    }
  
    get value(){
      return this._value;
    }
  
    set value(v){
      this._value = v;
    }
  
    get id(){
      return this._id;
    }
  
    set id(i){
      this._id = i;
    }
  
    get obj(){
      return this._obj;
    }
  
    set obj(o){
      this._obj = o;
    }
  }

  function isSmartPhone() {
    if (navigator.userAgent.match(/iPhone|Android.+Mobile/)) {
      return true;
    } else {
      return false;
    }
  }

  var poseX = 0;
  var poseY = 0;
  class stick{
    constructor(id){
      this._id = id;
      this._obj = document.getElementById(id);
      this._value = 0;
      this._x = 0;
      this._y = 0;
      this._startX = 0;
      this._startY = 0;
      document.getElementById("btnArea").addEventListener('mousemove',(mm)=>{
        if(isSmartPhone()){
          // mm.preventDefault();
        }
        else{
          poseX = mm.x/50;
          poseY = mm.y/50;
        }
        if(this.getValue()==1){
          var L = Math.sqrt((this.getstartX()-poseX)*(this.getstartX()-poseX) + (this.getstartY()-poseY)*(this.getstartY()-poseY));
          if(L>1){
            this.setX((this.getstartX()-poseX)/L);
            this.setY((this.getstartY()-poseY)/L);
          }else{
            this.setX(this.getstartX()-poseX);
            this.setY(this.getstartY()-poseY);
          }
        }
        else{
          this.setX(0);
          this.setY(0);
        }
        this._obj.style.left = (this.getX()*(-0.5)+0.5)*70+"%";
        this._obj.style.top = (this.getY()*(-0.5)+0.5)*70+"%";
      },false);
      document.getElementById("btnArea").addEventListener('touchmove',(mm)=>{
        if(isSmartPhone()){
          mm.preventDefault();
          poseX = mm.touches[0].pageX/50;
          poseY = mm.touches[0].pageY/50;
        }
        else{
          poseX = mm.touches[0].pageX;
          poseY = mm.touches[0].pageY;
        }
        if(this.getValue()==1){
          var L = Math.sqrt((this.getstartX()-poseX)*(this.getstartX()-poseX) + (this.getstartY()-poseY)*(this.getstartY()-poseY));
          if(L>1){
            this.setX((this.getstartX()-poseX)/L);
            this.setY((this.getstartY()-poseY)/L);
          }else{
            this.setX(this.getstartX()-poseX);
            this.setY(this.getstartY()-poseY);
          }
        }
        else{
          this.setX(0);
          this.setY(0);
        }
        this._obj.style.left = (this.getX()*(-0.5)+0.5)*70+"%";
        this._obj.style.top = (this.getY()*(-0.5)+0.5)*70+"%";
      },false);
      document.getElementById("btnArea").addEventListener('mouseup',()=>{this.setValue(0);
        this.setX(0);
        this.setY(0);
        this._obj.style.left = (this.getX()*(-0.5)+0.5)*70+"%";
        this._obj.style.top = (this.getY()*(-0.5)+0.5)*70+"%";},false);
      document.getElementById("btnArea").addEventListener('touchend',()=>{this.setValue(0);
        this.setX(0);
        this.setY(0);
        this._obj.style.left = (this.getX()*(-0.5)+0.5)*70+"%";
        this._obj.style.top = (this.getY()*(-0.5)+0.5)*70+"%";},false);
      document.getElementById("btnArea").addEventListener('touchcancel',()=>{this.setValue(0);
        this.setX(0);
        this.setY(0);
        this._obj.style.left = (this.getX()*(-0.5)+0.5)*70+"%";
        this._obj.style.top = (this.getY()*(-0.5)+0.5)*70+"%";},false);
      this._obj.addEventListener('mousedown',(mm)=>{this.setValue(1);
        poseX = mm.x/50;
        poseY = mm.y/50;
        this.setstartX();
        this.setstartY();},false);
      this._obj.addEventListener('touchstart',(mm)=>{this.setValue(1);
        poseX = mm.touches[0].pageX/50;
        poseY = mm.touches[0].pageY/50;
        this.setstartX();
        this.setstartY();},false);
      this._obj.addEventListener('mouseup',()=>{this.setValue(0);},false);
      this._obj.addEventListener('touchend',()=>{this.setValue(0);},false);
      this._obj.addEventListener('touchcancel',()=>{this.setValue(0);},false);
    }
  
    setValue(v){
      this._value = v;
    }
    setstartX(){
      this._startX = poseX;
    }
    setstartY(){
      this._startY = poseY;
    }
    setX(v){
      this._x = v;
    }
    setY(v){
      this._y = v;
    }

    getX(){
      return this._x;
    }
    getY(){
      return this._y;
    }
    getstartX(){
      return this._startX;
    }
    getstartY(){
      return this._startY;
    }
    getValue(){
      return this._value;
    }
  
    get value(){
      return this._value;
    }
  
    set value(v){
      this._value = v;
    }
  
    get id(){
      return this._id;
    }
  
    set id(i){
      this._id = i;
    }
  
    get obj(){
      return this._obj;
    }
  
    set obj(o){
      this._obj = o;
    }
  }
  
  var Id = -1;
  var usedId = [];
      window.onload = function(){
        setTimeout(() => {
          if(Id < 0){
            Id = 0;
          }
        },2000);
        let cross = new button('btn0');
        let circle = new button('btn1');
        let triangle = new button('btn2');
        let square = new button('btn3');
        let L1 = new button('btn4');
        let R1 = new button('btn5');
        let L2 = new button('btn6');
        let R2 = new button('btn7');
        let select = new button('btn8');
        let start = new button('btn9');
        let pairing = new button('btn10');
        let stickLeft = new stick('btn11');
        let stickRight = new stick('btn12');
        let crossUp = new button('btn13');
        let crossDown = new button('btn14');
        let crossLeft = new button('btn15');
        let crossRight = new button('btn16');
        // let stickLeftUp = new button('btn17');
        // let stickLeftDown = new button('btn18');
        // let stickLeftLeft = new button('btn19');
        // let stickLeftRight = new button('btn20');
        // let stickRightUp = new button('btn21');
        // let stickRightDown = new button('btn22');
        // let stickRightLeft = new button('btn23');
        // let stickRightRight = new button('btn24');
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
                  name : '/idcheck',
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
                    name : '/idcheck',
                    messageType : 'std_msgs/Int32'
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
                  name : '/joy',
                  messageType : 'sensor_msgs/Joy'
              });
              pub.publish(msg);
          },
          getIp : function(){
            if(document.getElementById("IP").value.length == 0) return;
            return document.getElementById("IP").value;
          },
          getJoy : function(ID){
            var joydata = new ROSLIB.Message({
              header : { frame_id : "webjoy"+ID },
              axes:[stickLeft.getX(),
              stickLeft.getY(),
              L2.getValue()*2-1,
              stickRight.getX(),
              stickRight.getY(),
              R2.getValue()*2-1,
              crossUp.getValue()-crossDown.getValue(),
              crossLeft.getValue()-crossRight.getValue()],
              buttons:[cross.getValue(),
              circle.getValue(),
              triangle.getValue(),
              square.getValue(),
              L1.getValue(),
              R1.getValue(),
              L2.getValue(),
              R2.getValue(),
              select.getValue(),
              start.getValue(),
              pairing.getValue(),
              stickLeft.getValue(),
              stickRight.getValue(),
              crossUp.getValue(),
              crossDown.getValue(),
              crossLeft.getValue(),
              crossRight.getValue()]
            });
            return joydata;
          }
        }
      Talker.init();
      
      setInterval(function (){
        joy_data = Talker.getJoy(Id);
        ip = Talker.getIp();
        //console.log(ip);
        if(Id>=0){
          Talker.send(joy_data,ip);
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
