// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.SimDevice;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
 
 
public class Robot extends TimedRobot {
  public Robot(){
    super(0.05);
  }
 
  private double mag, mag2;
  private double seno, seno2;
  private int pov;  
  private double spd = 1;
  private double mL = 0,mR = 0;
  private double mE = 0, mA = 0;
  private double x1,y1,x2,y2;
  private boolean analogic1, analogic2;
  private double rt, lt, rt2, lt2;
  private boolean a,b,x,y;
  private boolean ltB,rtB;
  private int encode;

  private double i = 0, j = 0;

  private final VictorSPX red = new VictorSPX(6);
  private final VictorSPX redS = new VictorSPX(3);
  private final VictorSPX left1 = new VictorSPX(2);
  private final VictorSPX left2 = new VictorSPX(7);
  private final VictorSPX right1 = new VictorSPX(1);
  private final VictorSPX right2 = new VictorSPX(4);
  private final Encoder parentalControl = new Encoder(8,9);
  private double mp = 0;
  private double spdAxis,yAxis = 0;

  // VictorSPX m_direita1 = new VictorSPX(1);
  // VictorSPX m_direita2 = new VictorSPX(2);
  // VictorSPX m_esquerda1 = new VictorSPX(3);
  // VictorSPX m_esquerda2 = new VictorSPX(4);
 
  //VictorSPX m_escalada = new VictorSPX(7);
  // VictorSPX m_angulo = new VictorSPX(6);
  
  Joystick js = new Joystick(0);
  Joystick moveC = new Joystick(1);
 
  Thread m_visionThread;
  double[][] vetpower = new double[700][2];
  String texto = "";
  int p,q;

  Accelerometer accelerometer = new BuiltInAccelerometer();
  Encoder m_encoder = new Encoder(0, 1);
  SimDevice davi = new SimDevice(encode);
 
 
 
  @Override
  public void robotInit() {
    parentalControl.setDistancePerPulse(0.25);
    red.setNeutralMode(NeutralMode.Brake);
    redS.setNeutralMode(NeutralMode.Brake);
    left2.follow(left1);
    right2.setNeutralMode(NeutralMode.Brake);
    right2.follow(right1);
    left2.setNeutralMode(NeutralMode.Brake);
    left1.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);  
    right1.setInverted(true);
    right2.setInverted(true);

    m_encoder.reset();
  }
 
  public void teleopInit(){
  }
 
  @Override
  public void teleopPeriodic()  {
 
    // Atribuição de valores as variaveis
 
      // Atribuição dos valores dos eixos - Analogico esquerdo
      x1 = moveC.getRawAxis(0); // Eixo X
      y1 = - moveC.getRawAxis(1); // Eixo Y
 
      x2 = moveC.getRawAxis(4);
      y2 = - moveC.getRawAxis(5);
     
      // Atribuição do valor do POV
      pov = moveC.getPOV();
 
      // Atribuição do valor dos gatilhos
      rt = moveC.getRawAxis(3);
      lt = - moveC.getRawAxis(2);
 
      a = moveC.getRawButton(1);
      b = moveC.getRawButton(2);
      x = moveC.getRawButton(3);
      y = moveC.getRawButtonPressed(4);
 
    // Calculo das magnitudes
      mag = Math.hypot(x1, y1);
      mag2 = Math.hypot(x2, y2);
 
    // Verificação do uso de botões
      buttonSe(x,a,b);
    // Verificação dos analogicos
      movementCalc();
    // Calculo dos triggers
      triggerCalc(rt,lt,x1);
    // Reiniciação dos valores insignificantes
      resetAxis();
    // Calculo do POV
      povCalc(pov);
      SmartDashboardF();
      left1.set(ControlMode.PercentOutput, mL);
      right1.set(ControlMode.PercentOutput, mR);

      yAxis = -js.getY();
      spdAxis = (js.getRawAxis(3) + 1)/2;   
      if(js.getRawButton(1)){
        yAxis = Math.round(yAxis*2);
        if(yAxis > 1) yAxis = 1;
        if(yAxis < -1) yAxis = -1;
      } 
      if(Math.abs(yAxis) > 0.1) 
        mp = yAxis * spdAxis;
      else 
        mp = 0;
      SmartDashboard.putNumber("MP", mp);
      SmartDashboard.putNumber("spdAxis", spdAxis);
      red.set(ControlMode.PercentOutput, mp);
  }
 
  private void SmartDashboardF() {
 
      //Movimentação
        SmartDashboard.putNumber("Forca Motor Esquerdo", minMethod(mL));
        SmartDashboard.putNumber("Força Motor Direito", minMethod(mR));
        SmartDashboard.putNumber("Magnitude Esquerda", mag);
        SmartDashboard.putNumber("Magnitude Direita", mag2);
        SmartDashboard.putNumber("Trigger Esquerdo", -lt);
        SmartDashboard.putNumber("Trigger Direito", rt);
        SmartDashboard.putString("Analogico ativo",analogicGate(analogic1,analogic2));
        SmartDashboard.putString("Trigger ativo", analogicGate(ltB,rtB));
 
      //Escalada
        SmartDashboard.putNumber("Trigger Direito 2", rt2);
        SmartDashboard.putNumber("Trigger Esquerdo 2", lt2);
        SmartDashboard.putNumber("Valor de I", i);
        SmartDashboard.putNumber("Valor de J", j);
        SmartDashboard.putNumber("Motor Escalada", mE);
        SmartDashboard.putNumber("Motor Angulacao", mA);
        SmartDashboard.putNumber("Valor de Q", q);
        SmartDashboard.putNumber("Valor de P", p);
        SmartDashboard.putNumber("Valor do Encoder", encode);
     
  }
 
  private String analogicGate(boolean a, boolean b) {
   
    if(a) return "Esquerdo"; // Verificação do uso do componente esquerdo
 
      else if(b) return "Direito"; // Verificação do uso do componente direito
 
        else return "Nenhum"; // Verificação da inutilização dos dois componentes
 
  }
 
  private void resetAxis() {
 
    // Verificação de inatividade dos analogicos
      if(mag < 0.1){
        x1 = 0;
        y1 = 0;
        mag = 0;
      }
      if(mag2 < 0.1){
        x2 = 0;
        y2 = 0;
        mag2 = 0;
      }
    // Verificação da inatividade de ambos analogicos
      if(mag < 0.1 && mag2 < 0.1 && rt==0 && lt == 0){
        mL = 0;
        mR = 0;
      }
 
    }
  
  //Função de movimentação dos gatilhos
  public void triggerCalc(double rt,double lt,double x){
      if(Math.abs(x) < 0.04) x = 0;
 
      if(rt != 0){
          rtB = true;
          ltB = false;

        if(x >= 0){
          mL = rt * spd;
          mR = rt * (1 - x) * spd;
        }else if(x < 0){
          mL = rt * (1 + x) * spd;
          mR = rt *spd;
        }
      }else if(lt != 0){
        ltB = true;
        rtB = false;
        if(x >= 0){
          mL = lt * (1 - x) * spd;
          mR = lt * spd;
        }else if(x < 0){
          mL = lt * spd;
          mR = lt * (1 + x) * spd;
      
      }
    }
  }
  
  // Função da Movimentação pelo Analógico 1
  public void quadCalc(double y, double x) {
      seno = y / mag;
      // Quadrante 1  
        if(y >= 0 && x >= 0){
          mR = (2 * seno - 1) * mag * spd; // Varia
          mL = mag * spd; // Constante
      // Quadrante 2
        }else if(y >= 0 && x <= 0){
          mR = mag * spd; // Constante
          mL = (2 * seno - 1) * mag * spd; // Varia
      // Quadrante 3
        }else if(y < 0 && x < 0){
          mR = -mag * spd; // Constante
          mL = (2 * seno + 1) * mag * spd; // Varia
      // Quadrante 4
        }else if(y < 0 && x >= 0){
          mR = (2 * seno + 1) * mag * spd; // Varia
          mL = -mag * spd; // Constante
        }
    }


  // Função de movimentação por POV (Botões Digitais)
  public void povCalc(int pov){
    // Calculo do POV
      switch(pov){
 
        case 0:
          mR = 0.25;
          mL = 0.25;
        break;
 
        case 45:
          mR = 0;
          mL = 0.25;
        break;
 
        case 90:
          mR = -0.25;
          mL = 0.25;
        break;
 
        case 135:
          mR = -0.25;
          mL = 0;
        break;
 
        case 180:
          mR = -0.25;
          mL = -0.25;
        break;
 
        case 225:
          mR = 0;
          mL = -0.25;
        break;
 
        case 270:
          mR = 0.25;
          mL = -0.25;
        break;
 
        case 315:
          mR = 0.25;
          mL = 0;
        break;
      }
  }
 
  // Função que detecta qual analógico esta sendo utilizado
  private void movementCalc(){
 
    //Verificação do analogico esquerdo
      if(minMethod(mag) != 0){
        analogic1 = true;
        analogic2 = false;
      // Calculo dos quadrantes
        quadCalc(y1, x1);
    }
 
      else if(minMethod(mag2)!=0){
      // Calculo dos quadrantes
        reverseQuadCalc();
        analogic1 = false;
        analogic2 = true;
    }
 
      else{
        analogic1 = false;
        analogic2 = false;
    }
}
  // Função do analógico (Reverso)
  private void reverseQuadCalc() {
      seno2 = y2 / mag2;
        // Quadrante 1  
          if(y2 >= 0 && x2 >= 0){
            mR = - mag2 * spd;
            mL = (- 2 * seno2 + 1) * mag2 * spd;
        // Quadrante 2
          }else if(y2 >= 0 && x2 < 0){
            mR = (- 2 * seno + 1) * mag2 * spd;
            mL = - mag2 * spd;
        // Quadrante 3
          }else if(y2 < 0 && x2 < 0){
            mR = (- 2 * seno - 1) * mag2 * spd;
            mL = mag2 * spd;
        // Quadrante 4
          }else if(y2 < 0 && x2 >= 0){
            mR = mag2 * spd;
            mL = (- 2 * seno - 1) * mag2 * spd;
      }
  }
 
  // Função de verificação dos vlaores dos botões
  private void buttonSe(boolean x,boolean a, boolean b){
   
    // Verificação dos botões
    if (x) // Força Máxima
      spd = 1;
   
      else if(a) // Força Média
        spd = 0.5;
 
        else if(b) // Força Mínima
          spd = 0.25;
 
          SmartDashboard.putNumber("Velocidade", spd);
  }

  //Método para arredondamento de valores insignificantes
  private double minMethod(double a){

    if(Math.abs(a) < 0.04) return 0;
      else return a;
    
  }
 
  // Função que trava os motores caso o botão Y do controle 2 seja pressionado
  private void lockMotors(boolean y){
    if(y){
      mR = 0;
      mL = 0;
    }
  }
 
  // Função que controla escalada
  // private double escalada(double rt, double lt){
 
  //   // Comando para calculo da potencia da velocidade
  //   double spd2 = buttonSe(buttonX2,a2,b2);
 
  //   //Altura máxima em 1 RPS por 0,05 segundos
  //   final int max = 60;
 
  //   //Verificação do gatilho esquerdo
  //   if(rt > 0){
 
  //     //Atribuindo força ao motor & aumentando o valor do nosso índice
  //     mE = spd2;
  //     i = i + spd2;
 
  //     //Verificação para caso o indíce ter atingido seu valor máximo
  //     if(i >= max){ 
 
  //       //Corrigindo i para caso ele tenha passado o valor máximo & travando o motor da escalada
  //       i = max;
  //       mE = 0;
  //     }
 
  //   //Verificação do gatilho direito e verificação para caso o índice não esteja no valor mínimo
  //   }else if(lt > 0 && i > 0){
 
  //     //Atribuindo força ao motor & diminuindo o valor do índice
  //     mE = -1;
  //     i = i - spd2;
 
  //   //Caso nenhuma das condições anteriores seja atingida  
  //   }else{
 
  //     // Se a condição anterior tenha sido atingido, o motor deve ter força 0
  //     mE = 0;
  //   }
 
  //   return mE * spd2;
  // }
 
}