#include "system_test_support.h"
#include "chassis_ramp.h"
#include <limits>
using namespace SystemSupport;
using namespace SystemConfig::Chassis;
namespace {
void productionReady() { useMotionDefaults = true; readyRobot(); }
void forward() { sticks(FORWARD.maximum, STRAFE.middle, ROTATE.middle); }
void centered() { sticks(FORWARD.middle, STRAFE.middle, ROTATE.middle); }
void assertBounds() {
    const auto c=ChassisController::status();
    for (uint8_t i=0;i<4;++i) {
        CHECK(std::isfinite(c.wheel[i]) && fabsf(c.wheel[i]) <= SAFETY_MAX_SPEED+0.001f);
        CHECK(std::isfinite(c.targetWheel[i]) && fabsf(c.targetWheel[i]) <= SAFETY_MAX_SPEED+0.001f);
    }
}
void assertPivot(const float (&wheel)[4], float x, float y) {
    const float vx=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])/4;
    const float vy=(-wheel[0]-wheel[1]-wheel[2]-wheel[3])/4;
    const float angular=(-wheel[0]-wheel[1]+wheel[2]+wheel[3])/4;
    const float k=(WHEELBASE_MM+TRACK_WIDTH_MM)/2;
    CHECK(fabsf(vx-angular*y/k)<0.002f);
    CHECK(fabsf(vy+angular*x/k)<0.002f);
}
void test(const std::string &name) {
    if(name=="motion_default_geometry_and_rates") {
        static_assert(WHEELBASE_MM==260 && TRACK_WIDTH_MM==320 && RAMP_ENABLED,"Measured defaults");
        productionReady(); const auto c=ChassisController::status();
        CHECK(c.motion.rampEnabled && c.motionEditable);
        CHECK(c.motion.acceleration==90 && c.motion.deceleration==180);
        CHECK(c.motion.icrXMm==0 && c.motion.icrLeftYMm==0 && c.motion.icrRightYMm==0);
        const auto wire=request("status");
        CHECK(wire.find("\"motion\":{\"available\":true,\"editable\":true,\"ramp_enabled\":true")!=std::string::npos);
        writeStatusFixture(".pio/host-tests/chassis_motion_status_fixture.json",wire);
    } else if(name=="motion_icr_four_wheel_pivots") {
        const float xy[4][2]={{130,-160},{130,160},{-130,-160},{-130,160}};
        for(uint8_t i=0;i<4;++i) {
            const ChassisMath::IcrGeometry g{260,320,xy[i][0],xy[i][1]}; float w[4]{};
            ChassisMath::mixWheels(FORWARD.middle,STRAFE.middle,ROTATE.maximum,w,4,g);
            CHECK(fabsf(w[i])<0.0001f); assertPivot(w,g.xMm,g.yMm);
            for(float v:w) CHECK(fabsf(v)<=90.001f);

        }
    } else if(name=="motion_icr_external_points_and_translation") {
        for(float x:{-5000.0f,0.0f,900.0f,5000.0f})
          for(float y:{-5000.0f,-160.0f,0.0f,160.0f,5000.0f}) {
            const ChassisMath::IcrGeometry g{260,320,x,y}; float w[4]{};
            ChassisMath::mixWheels(FORWARD.middle,STRAFE.middle,ROTATE.maximum,w,4,g);
            assertPivot(w,x,y);
            for(float v:w) CHECK(fabsf(v)<=90.001f);
            for(uint8_t stage=1;stage<=5;++stage) {
                float base[4]; ChassisMath::mixWheels(FORWARD.maximum,STRAFE.maximum,ROTATE.middle,base,stage);
                ChassisMath::mixWheels(FORWARD.maximum,STRAFE.maximum,ROTATE.middle,w,stage,g);
                for(uint8_t i=0;i<4;++i) CHECK(floatClose(w[i],base[i]));
                ChassisMath::mixWheels(FORWARD.maximum,STRAFE.minimum,ROTATE.maximum,w,stage,g);
                for(float v:w) CHECK(std::isfinite(v) && fabsf(v)<=90.001f);
            }
        }
    } else if(name=="motion_icr_invalid_fails_zero") {
        for(float bad:{0.0f,-1.0f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}) {
            ChassisMath::IcrGeometry g{bad,320,0,0}; float w[4]{1,2,3,4};
            ChassisMath::mixWheels(FORWARD.maximum,STRAFE.maximum,ROTATE.maximum,w,4,g);
            for(float v:w) CHECK(v==0);
        }
    } else if(name=="motion_ramp_exact_rates_and_zero_crossing") {
        float w[4]{},target[4]{-90,90,90,-90};
        ChassisRamp::advance(w,target,15,90,180);
        CHECK(floatClose(w[0],-1.35f) && floatClose(w[1],1.35f));
        for(int n=0;n<100;++n) ChassisRamp::advance(w,target,15,90,180);
        CHECK(w[1]==90);
        for(float &v:target)v=-v;
        ChassisRamp::advance(w,target,15,90,180); CHECK(floatClose(w[1],87.3f));
        bool zero=false,negative=false;
        for(int n=0;n<140;++n) {
            const float before=w[1]; ChassisRamp::advance(w,target,15,90,180);
            if(w[1]==0)zero=true;
            if(w[1]<0){CHECK(zero);negative=true;}
            CHECK(fabsf(w[1]-before)<=2.701f);
        }
        CHECK(negative && w[1]==-90);
    } else if(name=="motion_ramp_common_fraction_preserves_pivot") {
        const ChassisMath::IcrGeometry g{260,320,130,160}; float w[4]{},target[4]{};
        ChassisMath::mixWheels(FORWARD.middle,STRAFE.middle,ROTATE.maximum,target,4,g);
        for(int n=0;n<160;++n) {
            ChassisRamp::advance(w,target,15,90,180); assertPivot(w,130,160); CHECK(w[1]==0);
        }
        for(uint8_t i=0;i<4;++i)CHECK(floatClose(w[i],target[i]));
        for(float &v:target)v=0;
        for(int n=0;n<40;++n){ChassisRamp::advance(w,target,15,90,180);assertPivot(w,130,160);}
        for(float v:w)CHECK(v==0);
    } else if(name=="motion_actual_loop_acceleration_deceleration") {
        productionReady(); forward(); tick(15);
        float prior=ChassisController::status().wheel[1]; CHECK(prior>0 && prior<=4.501f);
        for(int n=0;n<75;++n) {
            tick(15);const auto c=ChassisController::status();
            CHECK(c.targetWheel[1]==90 && c.wheel[1]>=prior && c.wheel[1]-prior<=1.351f);
            prior=c.wheel[1];assertBounds();
        }
        CHECK(prior==90);centered();
        for(int n=0;n<40;++n) {
            tick(15);const auto c=ChassisController::status();
            CHECK(c.wheel[1]<=prior && prior-c.wheel[1]<=2.701f);prior=c.wheel[1];
        }
        CHECK(prior==0 && ChassisController::status().motionEditable);
    } else if(name=="motion_actual_loop_icr_and_cadence") {
        productionReady();auto settings=ChassisController::defaultMotionSettings();
        settings.icrXMm=130;settings.icrLeftYMm=160;CHECK(ChassisController::configureMotion(settings));
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.maximum);
        uint32_t lastAt[4]{};unsigned counts[4]{};size_t cursor=Mock::sent.size();
        for(int n=0;n<260;++n) {
            tick(5);const auto c=ChassisController::status();assertPivot(c.wheel,130,160);CHECK(c.wheel[1]==0);
            while(cursor<Mock::sent.size()) {
                const auto f=Mock::sent[cursor++];
                if(mksFrame(f)||(f.msg.id&31)!=0x0D)continue;
                for(uint8_t i=0;i<4;++i)if(f.bus==CAN_BUS[i]&&(f.msg.id>>5)==NODE_ID[i]) {
                    if(counts[i])CHECK(mockMillis-lastAt[i]==15);
                    lastAt[i]=mockMillis;++counts[i];
                    float v;memcpy(&v,f.msg.buf,4);CHECK(floatClose(v,c.wheel[i]));
                }
            }
        }
        for(auto n:counts)CHECK(n>=85);
        const auto c=ChassisController::status();CHECK(c.wheel[2]==90);
    } else if(name=="motion_pending_stage_updates_target_ramps_output") {
        productionReady();forward();advance(1200);
        CHECK(ChassisController::status().wheel[1]==90);
        request("stage 2");advance(20);
        const auto c=ChassisController::status();
        CHECK(c.directionStage==2 && c.targetWheel[1]==-90);
        CHECK(c.wheel[1]>0 && c.wheel[1]<90);
        CHECK(MechanismController::status().pendingStage==2 || MechanismController::status().stage==2);
    } else if(name=="motion_loop_delay_caps_acceleration") {
        productionReady();forward();tick(15);
        const float before=ChassisController::status().wheel[1];
        tick(200);CHECK(floatClose(ChassisController::status().wheel[1]-before,4.5f));
        assertBounds();
    } else if(name=="motion_clock_wrap") {
        productionReady();forward();tick(15);
        mockMillis=UINT32_MAX-20;tick(5);
        const float before=ChassisController::status().wheel[1];tick(30);
        CHECK(mockMillis<30);
        CHECK(floatClose(ChassisController::status().wheel[1]-before,2.7f));assertBounds();
    } else if(name=="motion_link_loss_bypasses_ramp") {
        productionReady();forward();advance(300);CHECK(ChassisController::status().wheel[1]>0);
        radio().linked=false;tick(5);
        const auto c=ChassisController::status();CHECK(c.mode==ChassisController::Mode::Locked);
        for(uint8_t i=0;i<4;++i)CHECK(c.wheel[i]==0 && c.targetWheel[i]==0);
    } else if(name=="motion_stop_bypasses_ramp") {
        productionReady();forward();advance(300);request("stop");
        CHECK(SystemSafety::stopped());for(float v:ChassisController::status().wheel)CHECK(v==0);
        CHECK(!ChassisController::configureMotion(ChassisController::defaultMotionSettings()));
    } else if(name=="motion_configuration_validation_stationary_guard") {
        productionReady();auto settings=ChassisController::defaultMotionSettings();
        for(float bad:{0.0f,-1.0f,10001.0f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
            settings.acceleration=bad;CHECK(!ChassisController::configureMotion(settings));
            CHECK(ChassisController::status().motion.acceleration==90);
        }
        settings=ChassisController::defaultMotionSettings();settings.icrXMm=5001;
        CHECK(!ChassisController::configureMotion(settings));
        settings.icrXMm=100;forward();advance(15);
        CHECK(!ChassisController::configureMotion(settings));centered();advance(500);
        CHECK(ChassisController::configureMotion(settings));
        CHECK(ChassisController::status().motion.icrXMm==100);
    } else if(name=="motion_usb_configuration_and_strict_rejection") {
        productionReady();request("chassis.motion 1 60 120 260 320 130 -350 700");
        const auto m=ChassisController::status().motion;
        CHECK(m.acceleration==60 && m.deceleration==120 && m.icrXMm==130 && m.icrLeftYMm==-350 && m.icrRightYMm==700);
        for(const char *cmd:{"chassis.motion","chassis.motion 1 90 180 260 320 0 0",
            "chassis.motion 1 0 120 260 320 0 0 0",
            "chassis.motion 1 90 180 0 320 0 0 0","chassis.motion 2 90 180 260 320 0 0 0",
            "chassis.motion 1 90 180 260 320 5001 0 0","chassis.motion 1 90 180 260 320 NaN 0 0",
            "chassis.motion 1 90 180 260 320 0.5 0 0","chassis.motion 1 90 180 260 320 0 0 extra",
            "chassis.motion 1 90 180 260 320 0 -500 500 extra",
            "chassis.motion 1 90 180 260 320 0 -500 5001"}) {
            request(cmd,false);CHECK(ChassisController::status().motion.icrXMm==130);
        }
        forward();advance(50);request("chassis.motion 1 90 180 260 320 0 -500 500",false);
        CHECK(ChassisController::status().motion.icrXMm==130);
    } else if(name=="motion_cylinder_pwm_concurrency") {
        productionReady();request("stage 2");completeMechanism();
        request("mode debug");request("debug.cylinder 1 6000");
        forward();advance(200);
        CHECK(CylinderController::status().active);
        CHECK(ChassisController::status().wheel[1]<0 && ChassisController::status().wheel[1]>-90);
        const auto writes=Mock::pwmWrites.size();advance(300);
        CHECK(Mock::pwmWrites.size()>writes);CHECK(CylinderController::status().active);
        request("stop");assertCylinderStopped();for(float v:ChassisController::status().wheel)CHECK(v==0);
    }  else if(name=="motion_directional_defaults_left_right_and_neutral") {
        productionReady();
        for(uint8_t stage=1;stage<=5;++stage) {
            for(int rotate:{ROTATE.maximum,ROTATE.minimum}) {
                const float v=ChassisMath::mapJoystick(rotate,ROTATE);
                const auto g=ChassisMath::turnIcr(v,260,320,80,-350,700);
                const float expected=v*ICR_LEFT_ROTATE_SIGN>0?-350.0f:700.0f;
                CHECK(g.xMm==80 && g.yMm==expected);
                float w[4];ChassisMath::mixWheels(FORWARD.middle,STRAFE.middle,rotate,w,stage,g);
                assertPivot(w,80,expected);
                for(float x:w)CHECK(fabsf(x)<=90.001f);
            }
        }
        for(int rotate:{ROTATE.middle-DEAD_ZONE,ROTATE.middle,ROTATE.middle+DEAD_ZONE}) {
            const auto g=ChassisMath::turnIcr(ChassisMath::mapJoystick(rotate,ROTATE),260,320,80,-350,700);
            CHECK(g.xMm==0 && g.yMm==0);
        }
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.maximum);advance(1800);
        auto c=ChassisController::status();assertPivot(c.wheel,0,0);assertBounds();
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.minimum);advance(1800);
        c=ChassisController::status();assertPivot(c.wheel,0,0);assertBounds();
        centered();advance(1800);
        for(float w:ChassisController::status().wheel)CHECK(w==0);
    } else if(name=="motion_directional_asymmetric_ram_and_validation") {
        productionReady();auto m=ChassisController::defaultMotionSettings();
        m.icrXMm=90;m.icrLeftYMm=-330;m.icrRightYMm=800;
        CHECK(ChassisController::configureMotion(m));
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.maximum);advance(1800);
        assertPivot(ChassisController::status().wheel,90,-330);
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.minimum);advance(1800);
        assertPivot(ChassisController::status().wheel,90,800);
        centered();advance(1800);
        for(float bad:{5001.0f,-5001.0f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}) {
            auto invalid=m;invalid.icrRightYMm=bad;CHECK(!ChassisController::configureMotion(invalid));
            invalid=m;invalid.icrLeftYMm=bad;CHECK(!ChassisController::configureMotion(invalid));
            CHECK(ChassisController::status().motion.icrRightYMm==800);
        }
        m.icrXMm=m.icrLeftYMm=m.icrRightYMm=0;CHECK(ChassisController::configureMotion(m));
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.maximum);advance(1800);
        assertPivot(ChassisController::status().wheel,0,0);

    } else if(name=="procession_profile_defaults_preserve_active_mechanism") {
        productionReady(); autoCompleteMoves=false; request("next"); advance(50);
        CHECK(mechanismBusy()); const auto before=MechanismController::status();
        const size_t sentAt=Mock::sent.size(), servoAt=Mock::servoWrites.size(), gpioAt=Mock::gpioWrites.size();
        button(14,true); advance(30);
        const auto rc=RcController::status(); const auto c=ChassisController::status();
        const auto after=MechanismController::status();
        CHECK(rc.processionMode && c.processionMode && c.directionStage==2);
        CHECK(c.motion.rampEnabled && c.motion.acceleration==90 && c.motion.deceleration==180);
        CHECK(c.motion.wheelbaseMm==260 && c.motion.trackWidthMm==320);
        CHECK(c.motion.icrXMm==0 && c.motion.icrLeftYMm==0 && c.motion.icrRightYMm==0);
        CHECK(after.stage==before.stage && after.pendingStage==before.pendingStage);
        CHECK(after.step==before.step && after.completed==before.completed && after.progress==before.progress);
        CHECK(std::string(after.phase)==before.phase && after.planVersion==before.planVersion);
        CHECK(Mock::servoWrites.size()==servoAt && Mock::gpioWrites.size()==gpioAt);
        for(size_t i=sentAt;i<Mock::sent.size();++i) CHECK(!mksFrame(Mock::sent[i]));
        CHECK(!SystemSafety::stopped());
        writeStatusFixture(".pio/host-tests/procession_status_fixture.json",request("status"));
    } else if(name=="procession_forward_and_strafe_match_ch10_direction") {
        productionReady(); button(14,true); forward(); advance(1600);
        auto c=ChassisController::status(); CHECK(c.processionMode && c.directionStage==2);
        float expected[4]; expectedOrientedWheels(2,1,0,0,0,expected);
        for(uint8_t i=0;i<4;++i) {
            CHECK(floatClose(c.targetWheel[i],expected[i]));
            CHECK(floatClose(c.wheel[i],expected[i]));
        }
        centered(); advance(1200); sticks(FORWARD.middle,STRAFE.maximum,ROTATE.middle); advance(1600);
        c=ChassisController::status(); expectedOrientedWheels(2,0,1,0,0,expected);
        for(uint8_t i=0;i<4;++i) {
            CHECK(floatClose(c.targetWheel[i],expected[i]));
            CHECK(floatClose(c.wheel[i],expected[i]));
        }
        CHECK(MechanismController::status().stage==4 && MechanismController::status().completed==0);
        assertBounds();

    } else if(name=="procession_switch_while_moving_preserves_ramp") {
        productionReady(); forward(); advance(1600);
        CHECK(ChassisController::status().wheel[1]==90);
        radio().channels[14]=2000; float before[4];
        for(uint8_t i=0;i<4;++i) before[i]=ChassisController::status().wheel[i];
        bool profileSeen=false; uint32_t profileAt=0;
        for(unsigned n=0;n<40;++n) {
            tick(); const auto c=ChassisController::status();
            CHECK(c.mode==ChassisController::Mode::Running);
            for(uint8_t i=0;i<4;++i) {
                CHECK(fabsf(c.wheel[i]-before[i])<=2.701f);
                before[i]=c.wheel[i];
            }
            if(c.processionMode) {
                if(!profileSeen){profileSeen=true;profileAt=mockMillis;}
                CHECK(c.directionStage==2);
                // RC selection updates immediately; wheel targets are produced on the next15ms tick.
                if(mockMillis-profileAt>=SEND_INTERVAL_MS) CHECK(c.targetWheel[1]==-90);
                CHECK(c.wheel[1]>0);
            }
            assertBounds();
        }
        CHECK(profileSeen && !ChassisController::status().motionEditable);
        advance(1800); CHECK(ChassisController::status().wheel[1]==-90);
        CHECK(MechanismController::status().stage==4 && mechanismReady());
    } else if(name=="procession_left_right_turns_use_independent_pivots") {
        productionReady(); button(14,true);
        auto settings=ChassisController::defaultMotionSettings(1);
        settings.icrXMm=120; settings.icrLeftYMm=180; settings.icrRightYMm=-180;
        CHECK(ChassisController::configureMotion(settings,1));
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.maximum); advance(1800);
        auto c=ChassisController::status(); CHECK(c.processionMode);
        assertPivot(c.targetWheel,120,180); assertPivot(c.wheel,120,180); assertBounds();
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.minimum); advance(1800);
        c=ChassisController::status();
        assertPivot(c.targetWheel,120,-180); assertPivot(c.wheel,120,-180); assertBounds();
        centered(); advance(1800);
        for(float w:ChassisController::status().wheel) CHECK(w==0);
    } else if(name=="procession_ram_banks_restore_independently") {
        productionReady(); auto normal=ChassisController::defaultMotionSettings(0);
        normal.acceleration=70; normal.icrXMm=20; normal.icrLeftYMm=-220; normal.icrRightYMm=330;
        CHECK(ChassisController::configureMotion(normal,0));
        button(14,true);
        auto c=ChassisController::status(); CHECK(c.processionMode && c.motion.icrXMm==0);
        auto procession=ChassisController::defaultMotionSettings(1);
        procession.acceleration=55; procession.icrXMm=140;
        procession.icrLeftYMm=210; procession.icrRightYMm=-210;
        CHECK(ChassisController::configureMotion(procession,1));
        button(14,false); advance(30);
        c=ChassisController::status(); CHECK(!c.processionMode);
        CHECK(c.motion.acceleration==70 && c.motion.icrXMm==20);
        CHECK(c.motion.icrLeftYMm==-220 && c.motion.icrRightYMm==330);
        button(14,true); c=ChassisController::status();
        CHECK(c.processionMode && c.motion.acceleration==55 && c.motion.icrXMm==140);
        CHECK(c.motion.icrLeftYMm==210 && c.motion.icrRightYMm==-210);
        CHECK(!ChassisController::configureMotion(normal,0));
        CHECK(c.motionEditable && !SystemSafety::stopped());
    } else if(name=="procession_usb_expected_bank_rejects_stale_profile") {
        productionReady();
        request("chassis.motion 1 60 120 260 320 20 -350 700");
        request("chassis.motion 1 70 130 260 320 30 -300 650 1",false);
        CHECK(ChassisController::status().motion.icrXMm==20);
        button(14,true);
        request("chassis.motion 1 70 130 260 320 30 -300 650",false);
        request("chassis.motion 1 70 130 260 320 30 -300 650 0",false);
        CHECK(ChassisController::status().motion.icrXMm==0);
        request("chassis.motion 1 70 130 260 320 125 175 -185 1");
        auto c=ChassisController::status();
        CHECK(c.processionMode && c.motion.acceleration==70 && c.motion.icrXMm==125);
        CHECK(c.motion.icrLeftYMm==175 && c.motion.icrRightYMm==-185);
        request("chassis.motion 1 70 130 260 320 999 175 -185 2",false);
        CHECK(ChassisController::status().motion.icrXMm==125);
        button(14,false); c=ChassisController::status();
        CHECK(!c.processionMode && c.motion.acceleration==60 && c.motion.icrXMm==20);
    } else if(name=="procession_newest_high_and_release_fallback_ch10") {
        productionReady(); button(10,true); completeMechanism();
        CHECK(MechanismController::status().stage==2);
        button(14,true); CHECK(ChassisController::status().processionMode);
        CHECK(MechanismController::status().stage==2);
        button(14,false);
        CHECK(!ChassisController::status().processionMode && RcController::status().selectedStage==2);
        CHECK(ChassisController::status().directionStage==2);
        button(14,true); button(11,true); completeMechanism();
        CHECK(!ChassisController::status().processionMode && MechanismController::status().stage==3);
        const auto before=MechanismController::status();
        button(11,false);
        CHECK(ChassisController::status().processionMode && RcController::status().processionMode);
        CHECK(MechanismController::status().stage==before.stage && MechanismController::status().pendingStage==0);
        CHECK(mechanismReady() && !SystemSafety::stopped());
    } else if(name=="procession_held_at_boot_selects_profile_after_unlock") {
        useMotionDefaults=true; initialize();
        radio().channels[12]=1000; radio().channels[14]=2000; advance(1800);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Running);
        CHECK(RcController::status().processionMode && ChassisController::status().processionMode);
        CHECK(ChassisController::status().directionStage==2);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        CHECK(mechanismReady() && !SystemSafety::stopped());
    } else if(name=="procession_stage1_ch14_retains_bird_prop") {
        productionReady(); button(14,true); CHECK(ChassisController::status().processionMode);
        stage1Switch(2000); completeMechanism();
        CHECK(RcController::status().stage1Mode && MechanismController::status().stage==1);
        CHECK(!RcController::status().processionMode && !ChassisController::status().processionMode);
        propRcPulse(5); completeMechanism();
        CHECK(MechanismController::status().prop==5 && MechanismController::status().completed==1);
        CHECK(!ChassisController::status().processionMode && ChassisController::status().directionStage==1);
        CHECK(Mock::servo[2]==propPulse(5,1,2) && Mock::servo[5]==propPulse(5,1,5));

    } else if(name=="procession_stage1_exit_requires_ch14_release") {
        productionReady(); stage1Switch(2000); completeMechanism();
        propRcPulse(5); completeMechanism(); CHECK(radio().channels[14]==2000);
        CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==5);
        stage1Switch(1000); completeMechanism(); advance(100);
        CHECK(MechanismController::status().stage==4 && !RcController::status().stage1Mode);
        CHECK(!RcController::status().processionMode && !ChassisController::status().processionMode);
        CHECK(ChassisController::status().directionStage==4);
        button(14,false); button(14,true);
        CHECK(RcController::status().processionMode && ChassisController::status().processionMode);
        CHECK(ChassisController::status().directionStage==2 && MechanismController::status().stage==4);
    } else if(name=="procession_invalid_selector_clears_and_recovers_override") {
        productionReady(); button(14,true); CHECK(ChassisController::status().processionMode);
        const size_t sentAt=Mock::sent.size(); const auto before=MechanismController::status();
        radio().channels[14]=3000; tick(); advance(20);
        CHECK(!RcController::status().processionMode && !ChassisController::status().processionMode);
        CHECK(ChassisController::status().directionStage==4);
        radio().channels[14]=2000; advance(SystemConfig::Rc::DEBOUNCE_MS+30);
        CHECK(RcController::status().processionMode && ChassisController::status().processionMode);
        CHECK(ChassisController::status().directionStage==2);
        CHECK(MechanismController::status().stage==before.stage && MechanismController::status().pendingStage==before.pendingStage);
        for(size_t i=sentAt;i<Mock::sent.size();++i) CHECK(!mksFrame(Mock::sent[i]));
        CHECK(!SystemSafety::stopped());
    } else if(name=="procession_stop_clears_ramped_velocity_immediately") {
        productionReady(); button(14,true);
        sticks(FORWARD.middle,STRAFE.middle,ROTATE.maximum); advance(300);
        bool moving=false; for(float w:ChassisController::status().wheel) moving=moving||w!=0;
        CHECK(moving); request("stop"); CHECK(SystemSafety::stopped());
        const auto c=ChassisController::status();
        CHECK(c.mode==ChassisController::Mode::Locked && c.emergencyLatched);
        for(uint8_t i=0;i<4;++i) CHECK(c.wheel[i]==0 && c.targetWheel[i]==0);
        assertCylinderStopped();
    } else if(name=="motion_axis_error_stops_ramped_commands") {
        productionReady();forward();advance(300);CHECK(ChassisController::status().wheel[1]>0);
        const auto error=heartbeat(0,8,2);ChassisController::receive(CAN_BUS[0],error);
        ChassisController::poll();
        CHECK(ChassisController::status().mode==ChassisController::Mode::Locked);
        for(float v:ChassisController::status().wheel)CHECK(v==0);
    } else throw std::runtime_error("Unknown motion case");
}
}
int main(int argc,char **argv) {
    try { CHECK(argc==2);test(argv[1]);printf("PASS %s\n",argv[1]);return 0; }
    catch(const std::exception &e){fprintf(stderr,"FAIL %s: %s\n",argc>1?argv[1]:"?",e.what());return 1;}
}
