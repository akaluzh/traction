  -- traction calculation, minimum time 
  procedure CalcTrac_MinTime(CalcID in number, MakeStops in integer:= 0, vOptim in integer:=0) is
    vdS number;    vdS0 number;        
    vS1 number;    vS2 number;
    vV1 number;    vV2 number;
    bs1 number;    bs2 number;
    bv1 number;    bv2 number;
    bt1 number;    bt2 number;
    ben1 number;   ben2 number;
       
    vModeTypeId        number;
    vSostavMassAllowed number;   
    vTractionForce     number;
    vBrakingForce      number;
    vResistForce       number;
    vForce             number;
    vCurrent           number;
    VelLimit           number;
    vReturnDist        number;
    vS_Mark number;
    vReturnCount       number;    
    vForceMin          number;     
    vForceMax          number;    
    vTractionModeMin   number;  
    vTractionModeMax   number;
    vEndPoint          number;
    vTime              number; 
    vdTime             number;
    vExtraTemp         number;  
    vVoltage           number;
    vdEnergy           number;
    vEnergy            number;
    vSpeedmin          number:=20;
    iUseBrake          number:=0;
  begin
    delete from calctraction_output where calc = CalcID;  
  
    vModeTypeId:=pac_constant.cMotion_Type('traction') ;
    select ci.train, ci.profile into vTrain, vProfileId from calc_inputdata ci where ci.id = CalciD;

    select min(p.startkm+0.1*p.startm) into vS1 from road_profile p where p.profile=vProfileId;
    insert into calctraction_output 
         values (CalcId, vS1, 0, 0, 0, 0, 0, 0, null, null, 0, seq_tractioncalc.nextval, 0, 0,  null, null);

    select o.axiskm+0.1*o.axism into vS1 from operation_points_temp o where o.profile=vProfileId and o.station=
                                     (select ci.point_from from calc_inputdata ci where ci.id=CalcID);

    select o.axiskm+0.1*o.axism into vEndPoint from operation_points_temp o where o.profile=vProfileId and o.station=
                                     (select ci.point_to from calc_inputdata ci where ci.id=CalcID);

    select l.model, lm.mass, lm.locomotive_type, lm.voltage into vLocModel, vLocMass, vLocType, vVoltage
      from train t, locomotive l, locomotive_model lm
      where t.locomotive=l.id and l.model=lm.id and t.id = vTrain;
    
    -- find the train mass (or specify the locomotive)
    vRoadTypeId:=Get_RoadType(vS1);

    vSostavMassAllowed := Get_Sostav_Mass(CalcId);
       select sum(v.mass) into vSostavMass from train_van tv, van v where tv.van=v.id and tv.train=vTrain;
       vSostavMass := vSostavMass+vLocMass;
      -- if vSostavMass > vSostavMassAllowed then return(null); end if;
   -- vSostavMass := vSostavMassAllowed;

    -- initial conditions
    vdS0:=0.1;
    vdS:=vdS0;
    vV1:=0;
    vS_Mark:=vS1;          
    vReturnCount:=0;
    VelLimit:=100;
    vTime:=0;
    vExtraTemp:=15;
    vEnergy:=0;

   -- solve the motion equation
    while (vS1<=vEndPoint)
    loop
      vS2:=round(vS1+vdS,-log(10,vdS));
      if vS2>635.5 then
        null;
      end if;  
      
   -- find the "maximum" and "minimum" modes
      vResistForce:=round(Get_Total_Resistance(vV1, vModeTypeId, vS1, false), 2);
      if vModeTypeId=pac_constant.cMotion_Type('traction') then
          vForceMin:=vResistForce;
          vForceMax:=-vResistForce;
          for c in (select * from traction_mode t where t.locmodel = vLocModel and t.recommended=1 order by t.sort_order desc)
            loop
              vTractionMode:=c.id;
              vForce:=abs(vResistForce-Get_Traction_Force(vV1)/(vSostavMass*9.81));
              if vForce<vForceMin then 
                vForceMin:=vForce; vTractionModeMin:=vTractionMode;
              end if;
              vForce:=Get_Traction_Force(vV1)/(vSostavMass*9.81)-vResistForce;
              if vForce>vForceMax then 
                vForceMax:=vForce; vTractionModeMax:=vTractionMode;
              end if;
             end loop;  
          vTractionMode:=null;    

   -- initial choice of the motion mode
          if (vV1<0.9*VelLimit) then                                                         
             vTractionMode:=vTractionModeMax;
          else  
            if vTractionModeMin is null then 
              vTractionMode:=null;  vModeTypeId:=pac_constant.cMotion_Type('stopway');
            else
              vTractionMode:=vTractionModeMin;
            end if;   
          end if;    
          vTractionModeMin:=null;  vTractionModeMax:=null;
      end if;    

      if vReturnCount>0 then
         for i in 1..vReturnCount
           loop
             vModeTypeId:=Get_PrevMotionMode(vModeTypeId);
           end loop;
        if vOptim = 1 then    
         -- mode correction at precedent steps to avoid braking  
         if vModeTypeId=pac_constant.cMotion_Type('brake') then
           begin
            select t.s1, t.v1, t.modetype, t.time_hours, t.energy into bS1, bV1, vModeTypeId, bt1, ben1 from calctraction_output t where t.id=
                  (select max(id) from calctraction_output tt 
                                 where tt.calc=Calcid 
                                   and tt.modetype in (pac_constant.cMotion_Type('traction'))
                                   and tt.s1>vS2-10);
            select min(v1) into bv2 from calctraction_output tt where tt.calc=Calcid and tt.s1>=bS1; 
              if bv2 > vSpeedmin then
                  vS1:=bS1; vV1:=bV1; vTime:=bt1; vEnergy:=ben1;
                  vModeTypeId:=pac_constant.cMotion_Type('stopway');  
                  vResistForce:=round(Get_Total_Resistance(vV1, vModeTypeId, vS1, false), 2);    
                  delete from calctraction_output t where t.calc=CalcId and t.s1>=vS1; commit;
                  vS2:=round(vS1+vdS,-log(10,vdS));
                  vReturnCount:=0;
              else
                  vModeTypeId:=pac_constant.cMotion_Type('stopway');
                  iUseBrake:=1;
              end if;  
             exception when no_data_found then 
            vModeTypeId:=pac_constant.cMotion_Type('stopway');
            iUseBrake:=1;
           end;  
         end if;
        end if; 
       end if;
      
   -- calculation of the traction force    
      if vModeTypeId=pac_constant.cMotion_Type('traction') then
          vTractionForce:=round(Get_Traction_Force(vV1)/(vSostavMass*9.81), 2);
          if vTractionForce is null then 
            vTractionForce:=0;
            vModeTypeId:=pac_constant.cMotion_Type('stopway');
            vTractionMode:=null;
          end if;  
      else
          vTractionForce:=0;
      end if;    

  -- calculation of the braking force  
      if vModeTypeId=pac_constant.cMotion_Type('brake') then
            vBrakingForce:=round(Get_Braking_Force(vV1), 2);  
            vdS:=0.1*vdS0;
            vS2:=round(vS1+vdS,-log(10,vdS));     
            vReturnCount:=0;            
      else vBrakingForce:=0;                   
      end if;            

  -- velocity calculation        
      vForce := vTractionForce - vResistForce - vBrakingForce;
      vV2:=greatest(2*120*vForce*vdS + vV1*vV1,0)**0.5;

  -- current calculation        
      if vModeTypeId=pac_constant.cMotion_Type('traction') then
        vCurrent:=nvl(Get_Current(vV1), 0);
      else vCurrent:=0;
      end if;  

  -- time and energy calculation        
      if vS2-vS1 = vdS and (vV1+vV2)<>0 then 
            vdTime:=(vS2-vS1)*2/(vV1+vV2);
      else vdTime:=0;
      end if;
      vExtraTemp:=Get_Temper(vCurrent, vExtraTemp, vdTime);
      vdEnergy:=vVoltage*vCurrent*vdTime/1000;             

      insert into calctraction_output 
            values (CalcId, 
                       vS1, 
                       vV1, 
                       vTractionForce, 
                       vResistForce, 
                       vForce, 
                       vBrakingForce, 
                       vCurrent,
                       vTractionMode,
                       vModeTypeId,
                       vTime,
                       seq_tractioncalc.nextval,
                       vExtraTemp,
                       vEnergy,
                       null, null);
       commit;  

  -- check for the allowed velocity  
      if MakeStops=1 then
        begin
        select nvl(min(vl.velocity),60) into VelLimit from v_velocity_limits vl where vl.calc = CalcId
           and vl.startpoint <=vS2 and vl.endpoint > vS2;
        select 0.7*nvl(min(vl.velocity),60) into vSpeedMin from v_velocity_limits vl where vl.calc = CalcId
           and vl.startpoint <=vS2 and vl.endpoint > vS2 and vl.velocity>0;    
        exception when no_data_found then
          VelLimit:=60; vSpeedMin:=VelLimit*0.7;
        end; 
      else
        begin
        select nvl(min(vl.velocity),60) into VelLimit from v_velocity_limits vl where vl.calc = CalcId
           and vl.startpoint <=vS2 and vl.endpoint > vS2 and vl.velocity>0;
        exception when no_data_found then
          VelLimit:=60;
        end;    
        vSpeedMin:=VelLimit*0.7; 
      end if;  

  -- correction of the motion mode if allowed velocity is exceeded
      if vV2>VelLimit then

        if vOptim=1 then
          if iUseBrake=0 then 
           vS_Mark:=vS2;
           vReturnDist:=0.5*(vV2**2-VelLimit**2)/(2*120*(Get_Braking_Force(vV2)+ Get_Total_Resistance(vV2, pac_constant.cMotion_Type('brake'), vS2, false)));
              if vReturnDist < 0.2*vdS then                                             
                 vReturnCount:=vReturnCount+1;
              else
                 vReturnCount:=vReturnCount+10;
              end if;   
              select cc.s1, cc.v1, cc.time_hours, cc.energy into vS2, vV2, vTime, vEnergy from (select * from calctraction_output t where t.calc=CalcID order by t.s1 desc) cc where rownum=1;
              delete from calctraction_output t where t.calc=CalcId and t.s1 >= vS2;
              commit;
           end if;   
           if iUseBrake=1 then   
              insert into calctraction_output values (CalcId, vS2, vV2, null, null, null, null, null, null, null, vTime, seq_tractioncalc.nextval, null, vEnergy, null, null); 
              vdS:=0.1*vdS0;
              vReturnDist:=greatest(vdS,Get_Braking_Length(CalcId, vV2, VelLimit, vS2));
              select st.s1, st.v1, st.time_hours, st.energy into bs1, bv1, bt1, ben1 from (select * from calctraction_output t where t.calc=Calcid and t.s1 < round(vS2-vReturnDist,2) order by s1 desc) st where rownum=1;
              select st.s1, st.v1, st.time_hours, st.energy into bs2, bv2, bt2, ben2 from (select * from calctraction_output t where t.calc=Calcid and t.s1 >= round(vS2-vReturnDist,2) order by s1) st where rownum=1;
              vS2:=round(vS2-vdS-vReturnDist,2);      
              vV2:=bv1+(bv2-bv1)*(vS2-bs1)/(bs2-bs1); 
              vTime:=bt1+(bt2-bt1)*(vS2-bs1)/(bs2-bs1);
              vEnergy:=ben1+(ben2-ben1)*(vS2-bs1)/(bs2-bs1);
              delete from calctraction_output t where t.calc=CalcId and t.s1 >= vS2;
              commit;
              vModeTypeId:=pac_constant.cMotion_Type('brake');
              vTractionMode:=null;
              vReturnCount:=0;
            end if;  
          vdTime:=0; 
        elsif vOptim=0 then 
          vS_Mark:=vS2;
          vReturnDist:=0.5*(vV2**2-VelLimit**2)/(2*120*(Get_Braking_Force(vV2)+
                                                       Get_Total_Resistance(vV2, pac_constant.cMotion_Type('brake'), vS2, false)));
          if vReturnDist < 0.2*vdS then                        -- 0.1*vdS                        
              vReturnCount:=vReturnCount+1;
              select cc.s1, cc.v1, cc.time_hours, cc.energy into vS2, vV2, vTime, vEnergy from (select * from calctraction_output t where t.calc=CalcID order by t.s1 desc) cc where rownum=1;
              delete from calctraction_output t where t.calc=CalcId and t.s1 >=vS1;
              commit;
          else
              insert into calctraction_output values (CalcId, vS2, vV2, null, null, null, null, null, null, null, vTime, seq_tractioncalc.nextval, null, vEnergy, null, null); 
              vReturnDist:=Get_Braking_Length(CalcId, vV2, VelLimit, vS2);
              vdS:=0.1*vdS0;
              select st.s1, st.v1, st.time_hours, st.energy into bs1, bv1, bt1, ben1 from (select * from calctraction_output t where t.calc=Calcid and t.s1 < round(vS2-vReturnDist,2) order by s1 desc) st where rownum=1;
              select st.s1, st.v1, st.time_hours, st.energy into bs2, bv2, bt2, ben2 from (select * from calctraction_output t where t.calc=Calcid and t.s1 >= round(vS2-vReturnDist,2) order by s1) st where rownum=1;
              vS2:=round(vS2-vdS-vReturnDist,2);      --  !!!!!!!!!!!!!!
              vV2:=bv1+(bv2-bv1)*(vS2-bs1)/(bs2-bs1); 
              vTime:=bt1+(bt2-bt1)*(vS2-bs1)/(bs2-bs1);
              vEnergy:=ben1+(ben2-ben1)*(vS2-bs1)/(bs2-bs1);
              delete from calctraction_output t where t.calc=CalcId and t.s1 >= vS2;
              commit;
              vModeTypeId:=pac_constant.cMotion_Type('brake');
              vTractionMode:=null;
          end if;  
          vdTime:=0; 
        end if;
      end if;

      if vS2>=vS_Mark then
        vReturnCount:=0;
        iUseBrake:=0;
        if vV2<=0.95*VelLimit then    
           vModeTypeId:=pac_constant.cMotion_Type('traction');
        end if;
      end if;
        
      if vModeTypeId <> pac_constant.cMotion_Type('brake') then
        if Vellimit>0 and VelLimit<=15 then vdS:=0.1*vdS0;
        else vdS:=vdS0;
        end if;  
      end if;

      if ((vV1+vV2) <> 0) and (vS2-vS1)=vdS then
        vTime:=vTime+vdTime;
        vEnergy:=vEnergy + vdEnergy + 5.5*vdTime;
      end if;  
      vS1:=vS2; vV1:=vV2;   

      -- allowance for the stop times
     begin 
      select nvl(t.time_minutes,0) into vdTime from v_stay_time t where t.calc=CalcId and t.loc=vS2; 
      vTime:=vTime+vdTime/60;
     exception when no_data_found then null;
     end;   

      -- check for cycling
     begin
       select t.s1 into bS1 from calctraction_output t where t.calc=CalcId 
         group by t.s1
         having count(t.id)>10;
       raise_application_error(-20001, 'Calculation failed at S='||bS1||'. Contact the support');  
     exception when no_data_found then null;
     end;    
      
    end loop;

  end; 
  
