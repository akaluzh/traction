  -- тяговый расчет, заданные перегонные времена
  procedure CalcTrac_ExactRSTime(CalcID in number, MakeStops in integer:= 0, vOptim in integer) is
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
    vS_Mark            number;
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
    vAverSpeed         number;  
    vAverSpeed0        number;
    vTime1             number:=0;
    vDist1             number:=0;
    vTimeRS            number;
    iUseBrake          integer:=0;
    vSpeedMin          number:=20;
    vCoeff             number:=0;
    
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
    
    -- определяем массу состава (или задаться серией электровоза для пассажирского поезда заданного веса или выбрать серию электропоезда)
    vRoadTypeId:=Get_RoadType(vS1);
    vSostavMassAllowed := Get_Sostav_Mass(CalcId)+vLocMass;
       select sum(v.mass) into vSostavMass from train_van tv, van v where tv.van=v.id and tv.train=vTrain;
       vSostavMass := vSostavMass+vLocMass;
      
    -- начальные условия
    vdS0:=0.1;
    vdS:=vdS0;
    vV1:=0;
    vS_Mark:=vS1;          
    vReturnCount:=0;
    VelLimit:=100;
    vTime:=0;
    vExtraTemp:=15;
    vEnergy:=0;
   
   -- решаем уравнение движения, разбиваем путь на перегоны
   
    for cRailSegments in (select t.Loc1 Pstart, t.Loc2 Pend, nvl(t.time_minutes,0) time_minutes
                          from v_move_time t where t.calc=CalcId)
    loop
     
     -- поиск средней скорости на перегоне
      vTimeRS:=0;
      vAverSpeed0:=(cRailSegments.Pend-cRailSegments.Pstart)/(cRailSegments.Time_Minutes/60);
      vTime1:=0; vDist1:=0;
      for tt in (select greatest(r.startpoint, cRailSegments.pstart) Pstart,  least(r.endpoint, cRailSegments.pend) Pend, r.velocity
                   from v_velocity_limits r 
                  where r.calc = CalcId and r.velocity>0 and r.velocity <=vAverSpeed0
                   and cRailSegments.Pstart<=r.startpoint and cRailSegments.Pend>=r.endpoint)
        loop
          vTime1:=vTime1+1.2*(tt.pend-tt.pstart)/tt.velocity;
          vDist1:=vDist1+(tt.pend-tt.pstart);
        end loop;          
      if vTime1>0 then 
         vAverSpeed0:= (cRailSegments.Pend-cRailSegments.Pstart-vDist1)/(cRailSegments.Time_Minutes/60-vTime1);
      end if;    
      
      vCoeff:=0;
      
     while round(vS1+vdS,-log(10,vdS)) <= cRailSegments.Pend
     loop             

      vS2:=least(round(vS1+vdS,-log(10,vdS)), cRailSegments.Pend);
      if vS2>722 then
        null;
      end if;  
     
   -- корректировка средней скорости 
      vAverSpeed:=(cRailSegments.pend-vS1)/(cRailSegments.time_minutes/60-vTimeRS);
        vTime1:=0; vDist1:=0;
        for tt in (select r.startpoint Pstart,  least(r.endpoint, cRailSegments.Pend) Pend, r.velocity
                    from v_velocity_limits r 
                   where r.calc = CalcId and r.velocity>0 and r.velocity <=vAverSpeed0
                     and r.startpoint>vS2 and cRailSegments.Pend>=r.endpoint)
         loop
            vTime1:=vTime1+1.2*(tt.pend-tt.pstart)/tt.velocity;
            vDist1:=vDist1+(tt.pend-tt.pstart);
          end loop;  

        vAverSpeed:= (cRailSegments.Pend-vS1-vDist1)/(cRailSegments.time_minutes/60-vTimeRS-vTime1);
        if vAverSpeed<=0 then vAverSpeed:=vAverSpeed0; end if;

   -- поиск "максимального" и "минимального" режимов
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
          
   -- первичный выбор режима движения
          if vV1 < (0.9+vCoeff)*least(VelLimit, vAverSpeed) then                                                         
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
  
   -- корректировка режима движения
      if vReturnCount>0 then
         for i in 1..vReturnCount
           loop
             vModeTypeId:=Get_PrevMotionMode(vModeTypeId);
           end loop;
           if vOptim=1 then  
             -- корректировка режима на предыдущих шагах для избежания торможения  
             if vModeTypeId=pac_constant.cMotion_Type('brake') then
               begin
                select t.s1, t.v1, t.modetype, t.time_hours, t.energy into bS1, bV1, vModeTypeId, bt1, ben1 from calctraction_output t where t.id=
                      (select max(id) from calctraction_output tt 
                                     where tt.calc=Calcid 
                                       and tt.modetype in (pac_constant.cMotion_Type('traction'))
                                       and tt.s1>greatest(vS2-(5-vCoeff*10), cRailSegments.Pstart) and tt.v1>vSpeedMin);
                select min(v1) into bv2 from calctraction_output tt where tt.calc=Calcid and tt.s1>=bS1; 
                  if bv2 > vSpeedmin then
                      vS1:=bS1; vV1:=bV1; vTime:=bt1; vEnergy:=ben1;
                      vModeTypeId:=pac_constant.cMotion_Type('stopway');  
                      vResistForce:=round(Get_Total_Resistance(vV1, vModeTypeId, vS1, false), 2);    
                      delete from calctraction_output t where t.calc=CalcId and t.s1>=vS1; commit;
                      vS2:=round(vS1+vdS,-log(10,vdS));
                      vReturnCount:=0;
                      begin
                         select vTime-t.time_hours into vTimeRS from calctraction_output t where t.calc=CalcID and t.s1=cRailSegments.Pstart;
                      exception when no_data_found then
                         vTimeRS:=0;
                      end;  
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
        
    -- расчет силы тяги    
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
     
    -- расчет тормозной силы  
      if vModeTypeId=pac_constant.cMotion_Type('brake') then
            vBrakingForce:=round(Get_Braking_Force(vV1), 2);  
           -- vdS:=0.1*vdS0;
            vS2:=round(vS1+vdS,-log(10,vdS));     
            vReturnCount:=0;            
      else vBrakingForce:=0;                   
      end if;            
      
    -- расчет скорости        
      vForce := vTractionForce - vResistForce - vBrakingForce;
      vV2:=greatest(2*120*vForce*vdS + vV1*vV1,0)**0.5;
      
    -- расчет тока  
      if vModeTypeId=pac_constant.cMotion_Type('traction') then
        vCurrent:=nvl(Get_Current(vV1), 0);
      else vCurrent:=0;
      end if;  

    -- расчет времени, энергии
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
                       vAverSpeed, null);
       commit;  
     
    -- проверка на допустимую скорость  
      if MakeStops=1 then
        begin
        select nvl(min(vl.velocity),60) into VelLimit from v_velocity_limits vl where vl.calc = CalcId
           and vl.startpoint <=vS2 and vl.endpoint > vS2;
        exception when no_data_found then
          VelLimit:=60;
        end;     
      else
        begin
        select nvl(min(vl.velocity),60) into VelLimit from v_velocity_limits vl where vl.calc = CalcId
           and vl.startpoint <=vS2 and vl.endpoint > vS2 and vl.velocity>0;
        exception when no_data_found then
          VelLimit:=60;
        end;    
      end if; 
        vSpeedMin:= 0.7*vAverSpeed; 
   
  -- корректировка режима при превышении допустимой скорости
      if vV2>least(VelLimit,(1.25+vCoeff)*vAverSpeed) and vS2>=vS_Mark then
        VelLimit:=least(VelLimit, (1.25+vCoeff)*vAverSpeed);
        vS_Mark:=vS2;
        vReturnDist:=0.5*(vV2**2-VelLimit**2)/(2*120*(Get_Braking_Force(vV2)+ Get_Total_Resistance(vV2, pac_constant.cMotion_Type('brake'), vS2, false)));
        
        if vOptim = 0 and vReturnDist < 0.1*vdS then                                              
              vReturnCount:=vReturnCount+1;
              select cc.s1, cc.v1, cc.time_hours, cc.energy into vS2, vV2, vTime, vEnergy from (select * from calctraction_output t where t.calc=CalcID order by t.s1 desc) cc where rownum=1;
              delete from calctraction_output t where t.calc=CalcId and t.s1 >=vS1;
              commit;
        elsif vOptim = 1 and iUseBrake=0 then 
              if vReturnDist < 0.2*vdS then                                             
                 vReturnCount:=vReturnCount+1;
              else
                 vReturnCount:=vReturnCount+10;
              end if;   
              select cc.s1, cc.v1, cc.time_hours, cc.energy into vS2, vV2, vTime, vEnergy from (select * from calctraction_output t where t.calc=CalcID order by t.s1 desc) cc where rownum=1;
              delete from calctraction_output t where t.calc=CalcId and t.s1 >= vS2;
              commit;
        elsif (vOptim=0 and vReturnDist >= 0.1*vdS) or (vOptim=1 and iUseBrake=1) then 
              insert into calctraction_output values (CalcId, vS2, vV2, null, null, null, null, null, null, null, vTime, seq_tractioncalc.nextval, null, vEnergy, null, null); commit;
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
       begin
         select vTime-t.time_hours into vTimeRS from calctraction_output t where t.calc=CalcID and t.s1=cRailSegments.Pstart;
       exception when no_data_found then
         vTimeRS:=0;
       end;  
      end if; 
      
 
      if vS2>=vS_Mark then
         vReturnCount:=0;
         iUseBrake:=0;
      end if;
        
      if vV2<=(0.95*VelLimit) and vS2>=vS_Mark then
        vModeTypeId:=pac_constant.cMotion_Type('traction');
        if Vellimit>15 then
           vdS:=vdS0;
        else 
           vdS:=0.1*vdS0;
        end if;   
      end if;
      
      if ((vV1+vV2) <> 0) and (vS2-vS1)=vdS then
        vTime:=vTime+vdTime;
        vTimeRS:=vTimeRS+vdTime;
        vEnergy:=vEnergy+vdEnergy + 5.5*vdTime;
      end if;  
      vS1:=vS2; vV1:=vV2;   

      -- учет времени на остановки
     begin 
      select nvl(t.time_minutes,0) into vdTime from v_stay_time t where t.calc=CalcId and t.loc=vS2; 
      vTime:=vTime+vdTime/60;
     exception when no_data_found then null;
     end;   
     
      -- проверка на зацикливание
     begin
       select t.s1 into bS1 from calctraction_output t where t.calc=CalcId 
         group by t.s1
         having count(t.id)>10;
       raise_application_error(-20001, 'Некорректное завершение расчета при S='||bS1||'. Cообщите об ошибке разработчику');  
     exception when no_data_found then null;
     end;    
     
      -- корректировка в случае превышения перегонного времени хода над заданным
     if vS2 = cRailSegments.Pend then
         if round(vTimeRS*60)-cRailSegments.Time_Minutes >= 1 and vCoeff<0.25 then
           select t.s1, t.v1, t.time_hours, t.energy into vS1, vV1, vTime, vEnergy 
                      from calctraction_output t where t.calc=CalcID and t.s1=cRailSegments.Pstart;
           vTimeRS:=0;
           vCoeff:=vCoeff+0.05;  
           delete from calctraction_output t where t.calc=Calcid and t.s1>=vS1;
           vdS:=0.1;         
           vModeTypeId:=pac_constant.cMotion_Type('traction');
           vS_Mark:=vS1;
         end if;
      end if;   
    end loop;
      
    
   end loop;

  end; 
  
