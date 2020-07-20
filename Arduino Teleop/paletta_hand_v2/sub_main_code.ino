void sub_main_code(float rec, unsigned int *buff)
{
//  if(print_data)
//  {
//    print_all();
//  }
  states();
  bclose = false;
  bopen = false;
  
  //if (bt_close && !contact)
  //{
  //  DynaSet(SERVO_R, rec, close_vel_fing, max_torque);
  //  gripper_state = "FLEXING";
  //  bt_close = false;
  //}
  send_pos = (unsigned int) rec;  

  DynaSet(SERVO_R, send_pos, close_vel_fing, max_torque);
  /*
  if (buff[1] - buff[0] > 830)
  { 
    DynaSet(SERVO_R, send_pos, close_vel_fing, max_torque);
    bclose = true;
  }

  if (buff[0] - buff[1] > 830)
  {
    DynaSet(SERVO_R, send_pos, close_vel_fing, max_torque);
    bclose = false;
  }
  
  if (bclose && buff[0] < 1000)
  {
    DynaSet(SERVO_L, lstop, close_vel_pal, max_torque);
    gripper_state = "READY TO BE EXTENDED";
  }

  if (gripper_state = "READY TO BE EXTENDED" && buff[0] > 1500)
  {
    DynaSet(SERVO_L, lstart, close_vel_pal, max_torque);
    gripper_state = " ";
  }
  */
  //if (bt_open)
  //{
  //  DynaSet(SERVO_R, rstart, open_vel, max_torque);
  //  DynaSet(SERVO_L, lstart, open_vel, max_torque);
  //  gripper_state = "EXTENDING";
  //  contact = false;
  //  bt_open = false;
  //  inc_torq = torq_thresh;
  //}

  //if (loadR >= torq_thresh && !contact && gripper_state == "FLEXING")
  //{
   // contact = true;
   // gripper_state="CONTACT";
   // DynaSet(SERVO_R, rstop, close_vel_fing, max_torque);
   // }
   
   
   //if (gripper_state == "CONTACT")
   //{
   //  delay(100);
   //  DynaSet(SERVO_R, rstop, close_vel_fing, max_torque);
   //  DynaSet(SERVO_L, lstop, close_vel_pal, max_torque);
   //  gripper_state = "READY TO BE EXTENDED";
   //}
}

void DynaSet(unsigned int id, unsigned int pos, unsigned int vel, unsigned int trq)
{
  ax12SetRegister2(id, 30, pos);
  ax12SetRegister2(id, 32, vel);
  ax12SetRegister2(id, 34, trq);
}
void Read_pos(unsigned int id, unsigned int id1)
{
  pos[0] = ax12GetRegister(id, 36, 2);
  pos[1] = ax12GetRegister(id1, 36, 2);
  gposR = pos[0];
  gposL = pos[1];
  buff[1] = buff[0];
}
//String Read_gripper_state()
//{
//  if (gposR >= (rstart - 100))
//  {
//    gripper_state = "FULL EXTENDED";
//  }
//  return gripper_state;
//}

void Read_load(unsigned int id, unsigned int id1)
{
  load[0] = ax12GetRegister(id, 40, 2);
  load[1] = ax12GetRegister(id1, 40, 2);
  loadR = load[0];
  loadL = load[1];
  if (load[0] < 0 || load[0] > 1023)
  {
    load[0] = 0;
    loadR = 0;
  }
  if (load[1] < 0 || load[1] > 1023)
  {
    load[1] = 0;
    loadL = 0;
  }
}

void states()
{
  Read_pos(SERVO_R, SERVO_L);
  Read_load(SERVO_R, SERVO_L); 

//  if(flag==2)
//  {
//    loadR = load[0];
//    loadL = load[1];
//    gposR = pos[0];
//    gposL = pos[1];
//    flag=0;
//  }
}

//
//void print_all()
//{
//  Read_load(SERVO_R, SERVO_L);
//  loadR = load[0];
//  loadL = load[1];
//  Read_pos(SERVO_R, SERVO_L);
//  gposR = pos[0];
//  gposL = pos[1];
//  //Serial.print("Gripper State : "); Serial.print(gripper_state); Serial.print(" ");
//  Serial.print("Motor Load1 : "); Serial.print(loadR); Serial.print(" ");
//  Serial.print("Motor Load2 : "); Serial.print(loadL); Serial.println(" ");
//  //Serial.print("Right Position : "); Serial.print(gposR); Serial.println(" ");
//  //Serial.print("Left Position : "); Serial.print(gposL); Serial.println(" ");
//  //Serial.print("Counter : "); Serial.print(count); Serial.print(" ");
//  //Serial.print("Torque : "); Serial.print(inc_torq); Serial.print(" ");
//  //Serial.print("Bt Close : "); Serial.print(bclose); Serial.print(" ");
//  //Serial.print("Bt Open  : "); Serial.print(bopen); Serial.println(" ");
//  //Serial.print("Inc Torque  : "); Serial.print(inc_torq); Serial.println(" ");
//
//}
