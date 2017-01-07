void FrontSw_poll(){ //This function is used to control the front switches
  // There's nothing to do if the transition flag is not set
  if(!frontSw2_transition && !frontSw1_transition)return; 
  
   switch (frontSw_switch) {
  case 0: //Page Turner
      frontSw2_transition = 0; // If toggleswitch is now high, the switch is now UP otherwise it is now DOWN
      if(digitalRead(frontSw2)) {
        noteOn(0xEF,0x00,0x7F);//Ch16 Pitch bend, Patch 0, 127
        } 
      else {
        noteOn(0xEF,0x00,0x40); //Ch16 Pitch bend, Patch 0, 64
        } 
      // if(!frontSw2_transition)return;
      frontSw1_transition = 0; 
      if(digitalRead(frontSw1)) {
        noteOn(0xEE,0x00,0x7F);//Ch15 Pitch bend, Patch 0, 127
        } 
      else {
        noteOn(0xEE,0x00,0x40); //Ch15 Pitch bend, Patch 0, 64
        }   
    delayMicroseconds(500);         
    break;
    
  case 1: //Yamaha Sysex 
    /*
    0x00 Intro A    0x08 Main A    0x10 Fill in AA    0x20 Ending A
    0x01 Intro B    0x09 Main B    0x11 Fill in BB    0x21 Ending B
    0x02 Intro C    0x0A Main C    0x12 Fill in CC    0x22 Ending C
    0x03 Intro D    0x0B Main D    0x13 Fill in DD    0x23 Ending D
                                   0x18 Break Fill    */ 
                                   
        frontSw2_transition = 0; // If toggleswitch is now high, the switch is now UP otherwise it is now DOWN
        if(digitalRead(frontSw2)) {
        MIDI_PORT.write(0xF0); //Exclusive status
        MIDI_PORT.write(0x43); //Yamaha ID
        MIDI_PORT.write(0x7E); //Style
        MIDI_PORT.write(0x00); //empty
        MIDI_PORT.write(0x21); //Switch No. (Ending B)
        MIDI_PORT.write(0x00); //0x00 = Off | 0x7F = On
        MIDI_PORT.write(0xF7); //End of Exclusive      
        } 
      else {        
        MIDI_PORT.write(0xF0); //Exclusive status
        MIDI_PORT.write(0x43); //Yamaha ID
        MIDI_PORT.write(0x7E); //Style
        MIDI_PORT.write(0x00); //empty
        MIDI_PORT.write(0x21); //Switch No. (Ending B)
        MIDI_PORT.write(0x7F); //0x00 = Off | 0x7F = On
        MIDI_PORT.write(0xF7); //End of Exclusive   
        } 
      // if(!frontSw2_transition)return;
      frontSw1_transition = 0; 
      if(digitalRead(frontSw1)) {
        MIDI_PORT.write(0xF0); //Exclusive status
        MIDI_PORT.write(0x43); //Yamaha ID
        MIDI_PORT.write(0x7E); //Style
        MIDI_PORT.write(0x00); //empty
        MIDI_PORT.write(0x12); //Switch No. (Fill in CC)
        MIDI_PORT.write(0x00); //0x00 = Off | 0x7F = On
        MIDI_PORT.write(0xF7); //End of Exclusive 
        } 
      else {
        MIDI_PORT.write(0xF0); //Exclusive status
        MIDI_PORT.write(0x43); //Yamaha ID
        MIDI_PORT.write(0x7E); //Style
        MIDI_PORT.write(0x00); //empty
        MIDI_PORT.write(0x12); //Switch No. (Fill in CC)
        MIDI_PORT.write(0x7F); //0x00 = Off | 0x7F = On
        MIDI_PORT.write(0xF7); //End of Exclusive 
        }  
    delayMicroseconds(500);         
    break;  
    
  case 2: //Program Change
      frontSw2_transition = 0; // If toggleswitch is now high, the switch is now UP otherwise it is now DOWN
      if(digitalRead(frontSw2)) {
          program_change--;
             if (program_change < 0){
              program_change++;
            }     
          noteOn(0xC0,0x00,program_change); 
      }  
      else {
          // do nothing
          }
      
      // if(!frontSw2_transition)return;
      frontSw1_transition = 0; 
  
      if(digitalRead(frontSw1)) {
          program_change++;
            if (program_change > 127){
              program_change = 0;
            } 
          noteOn(0xC0,0x00,program_change);          
          }
      else {
          //do nothing
          }
        delayMicroseconds(500);   
        break;
   }
}



/*
  if(digitalRead(frontSw2)) {
      if (!PAGETURN_MODE){
      program_change--;
         if (program_change < 0){
          program_change++;
        }     
      noteOn(0xC0,0x00,program_change);   
      }
      else {
      //  Ch5, Patch 0, Piano
      noteOn(0xEF,0x00,0x7F);
      }
      } 
  else {
      if (!PAGETURN_MODE){
      // do nothing
      }
      else {
      noteOn(0xEF,0x00,0x40);
      }
     } 
  // if(!frontSw2_transition)return;
  frontSw1_transition = 0; 
  if(digitalRead(frontSw1)) {
      if (!PAGETURN_MODE){
      program_change++;
        if (program_change > 127){
          program_change = 0;
        } 
      noteOn(0xC0,0x00,program_change);          
      }
      else {
      //  Ch5, Patch 0, Piano
      noteOn(0xEE,0x00,0x7F);
      }
    } 
  else {
      if (!PAGETURN_MODE){
      //do nothing
      }
      else {    
      noteOn(0xEE,0x00,0x40);
      } 
     }
  delayMicroseconds(500);      
  } // EndOf PageTurn_poll
  */
