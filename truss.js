var jctr=0,mctr=0,sctr=0,fctr=0; // joint, member,support, force counter;
var fx,fy,fn;

var anx = [];  // array for node x coords
var any = [];  // array for node y coords

var ams = [];  // array for member start
var ame = [];  // array for member end

var E = 3000000;

var A = 2;

var EA = E*A;

var fixeddof = [];




var c = document.getElementById("myCanvas");
var ctx = c.getContext("2d");

//float fixed
function ffixed(x) { 
   
   var num = x;
   var n   = num.toFixed(2);
   var n = parseFloat(n, 10);

   return n;


}



function addjoint(){

console.log("adding Joint");

jctr++;

var nx,ny;



nx = document.getElementById("nx").value;

ny = document.getElementById("ny").value;

nx = 100*nx+300;   //required to adjust with canvas axes.

ny = -100*ny+300;

anx.push(nx);
any.push(ny);

console.log(nx,ny);

ctx.fillStyle = " #ffcc00";
ctx.fillRect(nx, ny, 7, 7);
ctx.fillText("joint "+jctr, nx, ny);

}


function addmember(){

console.log("adding member");

mctr++;

var ms, me ; //ms is member start me is member end



ms = document.getElementById("ms").value;
me = document.getElementById("me").value;

ams.push(ms-1);
ame.push(me-1);

ctx.fillStyle = "#404040";
ctx.beginPath();
ctx.moveTo(anx[ms-1], any[ms-1]); 
ctx.lineTo(anx[me-1], any[me-1]);
ctx.lineWidth = 4;
ctx.stroke();

}


function addsupport(){

sctr++;


 var sf = document.getElementById("fixed").checked;
 var sr = document.getElementById("roller").checked;
 var sn = document.getElementById("sn").value

 sn = parseFloat(sn);

  if(sf){

   ctx.fillStyle = "#0033cc";
   ctx.fillRect(anx[sn-1]-5,any[sn-1]+5, 15, 15); 

   fixeddof.push(sn*2-2);  //x dof
   fixeddof.push(sn*2-1);  //y dof

   }

  if(sr){  
  
  ctx.strokeStyle = "#00b300";
  ctx.beginPath();
  ctx.arc(anx[sn-1]+2,any[sn-1]+15, 7, 0, 2 * Math.PI);
  ctx.stroke();

  fixeddof.push(sn*2-1); // in rolling only y dof is fixed

  console.log(fixeddof);

  }
}








 function addforce(){

   fx = document.getElementById("fx").value;
   fy = document.getElementById("fy").value;
   fn = document.getElementById("fn").value;
      
      ctx.strokeStyle = "#FF0000";
      ctx.beginPath();  
	    ctx.moveTo(anx[fn-1], any[fn-1]);
      ctx.lineTo(anx[fn-1]+(fx/2),any[fn-1]+(-fy/2));
      ctx.lineWidth = 3;
      ctx.stroke();
 
        //adding boundary counditions
        //adding force to force vector 

      


 

 }


  function analyse(){

    

  
   console.log("hello world"); 
   console.log(jctr);

   //degree of freedom = 2 x no of nodes 

   var dof = 2*jctr;

   console.log(dof);

   // displacement & force vector with array of length DOF.


   // var displacement = Vector.Zero(dof); 

    //var force = Vector.Zero(dof);

   var force = [];     //creating a column vector of dim 1xdof

    for(f=0;f<dof;f++){

       force[f] = 0;
    }  

    console.log(force);

     //adding fx to odd dof. 

        force[(fn*2-2)]= parseFloat(fx);

        force[(fn*2-1)]= parseFloat(fy);

        console.log(force);



   




   
   //initialising Global system stiffness matrix. 

    var A = Matrix.Zero(dof,dof);

    var KS = [];

    for(var i=0;i<dof;i++){


         KS[i] = [];
   
        for(var j=0;j<dof;j++)
        {
          KS[i][j]=0;

        }



    }
  

    console.log(KS);



    console.log(A.inspect());

  
   // calculating local stiffness matrix. 

   for(var i=0; i<mctr; i++)
   {

      
      var xa = anx[ame[i]]-any[ams[i]];  // delta X = member end x - member start x 

      var ya = any[ame[i]]-any[ams[i]]   // delta Y = """"""

      console.log(xa,ya);

      // calculating length 


      var l = (Math.sqrt((xa*xa)+(ya*ya)));

      

      var C = xa/l;

      var S = ya/l;

      console.log(C,S); 

      //initialising local Kstiffness matrix 

      var K = [];

          K[0] = [C*C,C*S,-C*C,-C*S];
          K[1] = [C*S,S*S,-C*S,-S*S];
          K[2] = [-C*C,-C*S,C*C,C*S];
          K[3] = [-C*S,-S*S,C*S,S*S];

       
                       

       console.log(K);

       var EAL = EA/l;

       K = math.multiply(EAL,K);


              

      var edof = [ams[i]*2+1,ams[i]*2+2,ame[i]*2+1,ame[i]*2+2];

      console.log(edof);

      for(var i2=0;i2<4;i2++)
      {

        for(var j=0;j<4;j++)
        {

          console.log(edof[i2]-1,edof[j]-1);

          var dofx = edof[i2]-1;
          var dofy = edof[j]-1;

          KS[dofx][dofy] = KS[dofx][dofy] + K[i2][j];


        }


      }


      console.log(KS); //final stiffness matrix 

   }

   for(var i2=0;i2<dof;i2++)
      {

        for(var j=0;j<dof;j++)
        { 
           KS[i2][j] = ffixed(KS[i2][j]);

        }

      }

      console.log(KS);



    
  
  

    console.log(fixeddof);

    dofarr = [];

    for(var i=0;i<dof;i++){

      dofarr.push(i);
    }



   
    let difference = dofarr.filter(x => !fixeddof.includes(x));



  
    console.log(difference);


    var A = [];

    for(var i=0;i<difference.length;i++)
    {

      A[i]=[];

      for(var j=0;j<difference.length;j++)
      {



        A[i][j] = KS[difference[i]][difference[j]];


      }



    }

    console.log(A);

    var B = [];

    for(var i=0;i<difference.length;i++)
    {

       B.push(force[i]);

    }
    
    console.log(B);

    var disp;    // solving matrix eqn A*x = B 

    disp = math.lusolve(A,B);     

    console.log(disp);



    // calculating stress at each element

   for(var i=0; i<mctr; i++)
   {

      
      var xa = anx[ame[i]]-any[ams[i]];  // delta X = member end x - member start x 

      var ya = any[ame[i]]-any[ams[i]]   // delta Y = """"""

      console.log(xa,ya);

      // calculating length 


      var l = (Math.sqrt((xa*xa)+(ya*ya)));

      

      var C = xa/l;

      var S = ya/l;

      console.log(C,S); 


      var CM = [-C,-S,C,S];
       

       var EAL = EA/l;

     //  K = math.multiply(EAL,K);


     var Edisp=[];


              

      var edof = [ams[i]*2+1,ams[i]*2+2,ame[i]*2+1,ame[i]*2+2];

      console.log(edof);

      for(var i2=0;i2<4;i2++)
      {


        if(disp[edof[i2]-1])
        {

           Edisp.push(disp[edof[i2]-1][0]);

        }
          else
            Edisp.push(0);

      }

      
      console.log(Edisp);

      var EK = math.multiply(CM,Edisp);

      var EF = math.multiply(EAL,EK);


      console.log("streess at element"+i+1);

      console.log(EF);


      var rstr; 

      rstr = "Stress at element "+(i+1)+" = "+EF+" N.";
      
      document.getElementById("result").innerHTML += rstr;

  }  


  }

