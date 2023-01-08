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
  
  var Id = -1;
  var usedId = [];
      window.onload = function(){
        setTimeout(() => {
          if(Id < 0){
            Id = 0;
          }
        },2000);
        let btn0 = new button('btn0');
        let btn1 = new button('btn1');
        let btn2 = new button('btn2');
        let btn3 = new button('btn3');
        let btn4 = new button('btn4');
        let btn5 = new button('btn5');
        let btn6 = new button('btn6');
        let btn7 = new button('btn7');
        let btn8 = new button('btn8');
        let btn9 = new button('btn9');
        let btn10 = new button('btn10');
        let btn11 = new button('btn11');
        let btn12 = new button('btn12');
        let btn13 = new button('btn13');
        let btn14 = new button('btn14');
        let btn15 = new button('btn15');
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
              this.ros.connect('ws://' + location.hostname + ':9090');
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
              var sub3 = new ROSLIB.Topic({
                ros : this.ros,
                name : '/topic_status',
                messageType : 'std_msgs/String'
            });
            sub3.subscribe(function(message) {
              console.log(message.data);
              document.getElementById('textArea').textContent = message.data;
            });
          },
          send : function(msg,ip){
              var pub = new ROSLIB.Topic({
                  ros : this.ros,
                  name : '/additional',
                  messageType : 'sensor_msgs/Joy'
              });
              pub.publish(msg);
          },
          getIp : function(){
            if(document.getElementById("IP").value.length == 0) return location.hostname;
            return document.getElementById("IP").value;
          },
          getJoy : function(ID){
            var joydata = new ROSLIB.Message({
              header : { frame_id : "additional"+ID },
              axes:[],
              buttons:[btn0.getValue(),
              btn1.getValue(),
              btn2.getValue(),
              btn3.getValue(),
              btn4.getValue(),
              btn5.getValue(),
              btn6.getValue(),
              btn7.getValue(),
              btn8.getValue(),
              btn9.getValue(),
              btn10.getValue(),
              btn11.getValue(),
              btn12.getValue(),
              btn13.getValue(),
              btn14.getValue(),
              btn15.getValue()]
            });
            return joydata;
          }
        }
      Talker.init();
      
      setInterval(function (){
        joy_data = Talker.getJoy(Id);
        ip = Talker.getIp();
        // console.log(ip);
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