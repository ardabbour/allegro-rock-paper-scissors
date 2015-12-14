import fingertracker.*;                         // Import the 'fingertracker' library.
import SimpleOpenNI.*;                          // Import the 'SimpleOpenNI' library.
PrintWriter output;                             //
FingerTracker fingers;                          // Call the 'fingers' class.
SimpleOpenNI kinect;                            // Call the 'kinect' class.
                                                //
int threshold = 625;                            // Define the depth at which recognition will
                                                // occur.
int x1;                                         // Define values to hold in x-coordinate of a
                                                // vector.
int y1;                                         // Define values to hold in y-coordinate of a
                                                // vector.
int n;                                          // Define a counter.
int n2;                                         // Define a second counter
int avgx;                                       //
int avgy;                                       //
int sumx = 0;                                   //
int sumy = 0;                                   //
char result = 0;                                //
int xrange[] = new int [500];                   // Define an array from which to calculate the
                                                // average of the x-coordinates.
int yrange[] = new int [500];                   // Define an array from which to calculate the
                                                // average of the y-coordinates.
PFont f;                                        //
void setup() {                                  //
  size(640, 480);                               // Define size of video window.
  f = createFont("Arial", 16, true);            //
  output = createWriter("gesture.txt");         //
  kinect = new SimpleOpenNI(this);              // Initialize the Microsoft Kinect v1.
  kinect.setMirror(true);                       // Invert the video vertically.
  kinect.enableDepth();                         // Enable the depth stream.
  fingers = new FingerTracker(this, 640, 480);  // Initialize the finger tracker.
}                                               //
                                                //
void draw() {                                   //
  kinect.update();                              //
  background(255);                              //
  textFont(f, 16);                              //
  fill(0);                                      //
  PImage depthImage = kinect.depthImage();      // Define the depth video stream.
  image(depthImage, 0, 0);                      // Show the depth video stream.
  fingers.setThreshold(threshold);              // Set the depth to be used for recognition.
                                                //
  int[] depthMap = kinect.depthMap();           // 
                                                //
  fingers.update(depthMap);                     // 
                                                //
  int numcontours = fingers.fc.getNumContours();// Get the number of contours to be used.
  stroke(50,255,0);                             //
  for (int k = 0; k < numcontours; k++) {       // Draw the contour.
    fingers.fc.drawContour(k);                  //
  }                                             //
                                                //
  int numfingers = fingers.getNumFingers();     // Get the number of fingers detected.
  int arrayx[] = new int [numfingers];          // Define an array to store the x-coordinates of
                                                // all the fingers.
  int arrayy[] = new int [numfingers];          // Define an array to store the y-coordinates of
                                                // all the fingers.
  fill(255,0,0);                                //
  for (int i = 0; i < numfingers; i++) {        // Gesture recognition algorithim starts here.
    int x = (int)fingers.getFingerX(i);         //
    int y = (int)fingers.getFingerY(i);         //
    arrayx[i] = x;                              //
    arrayy[i] = y;                              //
    ellipse(x-5, y -5, 10, 10);                 //
    n++;                                        //
    if (n == numfingers) {                      //
      x1 = max(arrayx) - min(arrayx);           //
      y1 = max(arrayy) - min(arrayy);           //
      while(n2 < 500) {                         //
        xrange[n2] = x1;                        //
        yrange[n2] = y1;                        //
        sumx = sumx + xrange[n2];               //
        sumy = sumy + yrange[n2];               //
        n2++;                                   //
    }                                           //
                                                //
      n2 = 0;                                   //
      avgx = round(sumx/500);                   //
      avgy = round(sumy/500);                   //
      sumx = 0;                                 //
      sumy = 0;                                 //
      if(avgx < 100 && avgy < 100) {            // Is Rock detected?
        result = '1';                           //
        output = createWriter("gesture.txt");   //
        output.print(result);                   //
        output.flush();                         //
        output.close();                         //
        fill(255);                              //
        textSize(48);                           //
        textAlign(CENTER);                      //
        text("Rock detected!", width/2, height/4);
      }                                         //
      else if(avgx > 100 && avgy > 100) {       // Is Paper detected?
        result = '2';                           //
        output = createWriter("gesture.txt");   //
        output.print(result);                   //
        output.flush();                         //
        output.close();                         //
        fill(255);                              //
        textSize(48);                           //
        textAlign(CENTER);                      //
        text("Paper detected!", width/2, height/4);
      }                                         //
      else if(avgx < 100 && avgy > 100) {       // Is Scissors detected?
        result = '3';                           //
output = createWriter("gesture.txt");           //
        output.print(result);                   //
        output.flush();                         //
        output.close();                         //
        fill(255);                              //
        textSize(48);                           //
        textAlign(CENTER);                      //
        text("Scissors detected!", width/2, height/4);
      }                                         //
      else {                                    // Condition if gesture is neither rock, paper 
        result = 0;                             // or scissor.
        output = createWriter("gesture.txt");   //
        output.print(result);                   //
        output.flush();                         //
        output.close();                         //
        fill(255);                              //
        textSize(48);                           //
        textAlign(CENTER);                      //
        text("Gesture not recognized!", width/2, height/4);
      }                                         //
      n = 0;                                    // Reset the counter.
    }                                           //
  }                                             //
}                                               //
                                                //
/*void keyPressed() {                           // Function to record selected gesture at
                                                // the press of a button.
    if(key == 'g'){                             //
      output = createWriter("gesturestore.txt");//
      output.print(result);                     //
      output.flush();                           //
      output.close();                           //
    }                                           //
}                                               //*/
