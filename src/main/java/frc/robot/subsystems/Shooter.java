package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Shooter extends SubsystemBase {
    private static Shooter m_Shooter;

    private SparkFlex m_leftMotor;
    private SparkFlex m_rightMotor;

    private SparkFlexConfig m_leftConfig;

    private double m_shootP;
    private double m_shootI;
    private double m_shootD;
    private double m_shootKv;
    private double m_shootKs;
    private TDNumber m_TDshootP;
    private TDNumber m_TDshootI;
    private TDNumber m_TDshootD;
    private TDNumber m_TDshootKv;
    private TDNumber m_TDshootKs;

    private SimpleMotorFeedforward m_Feedforward;

    private TDNumber m_TDvelocity;
    private TDNumber m_TDmeasuredVelocity;
    private TDNumber m_TDmeasuredCurrent;

    private Shooter() {
        super("Shooter");

        m_leftMotor = new SparkFlex(Constants.ShooterConstants.kLeftCANId, MotorType.kBrushless);
        m_rightMotor = new SparkFlex(Constants.ShooterConstants.kRightCANId, MotorType.kBrushless);

        m_TDshootP = new TDNumber(this, getName(), "P");
        m_TDshootI = new TDNumber(this, getName(), "I");
        m_TDshootD = new TDNumber(this, getName(), "D");
        m_TDshootKv = new TDNumber(this, getName(), "kV");
        m_TDshootKs = new TDNumber(this, getName(), "kS");

        m_TDshootP.set(Constants.ShooterConstants.kP);
        m_TDshootI.set(Constants.ShooterConstants.kI);
        m_TDshootD.set(Constants.ShooterConstants.kD);
        m_TDshootKv.set(Constants.ShooterConstants.kV);
        m_TDshootKs.set(Constants.ShooterConstants.kS);

        m_shootP = m_TDshootP.get();
        m_shootI = m_TDshootI.get();
        m_shootD = m_TDshootD.get();
        m_shootKv = m_TDshootKv.get();
        m_shootKs = m_TDshootKs.get();

        m_leftConfig = new SparkFlexConfig();
        m_leftConfig.closedLoop.pid(m_shootP, m_shootI, m_shootD);
        m_leftConfig.encoder.velocityConversionFactor(Constants.ShooterConstants.kVelocityFactor);

        SparkFlexConfig rightConfig = new SparkFlexConfig();
        rightConfig.follow(Constants.ShooterConstants.kLeftCANId);
        rightConfig.inverted(true);
        m_rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_TDvelocity = new TDNumber(this, getName(), "Target Velocity");
        m_TDvelocity.set(0);

        m_Feedforward = new SimpleMotorFeedforward(m_shootKs, m_shootKv);

        m_TDmeasuredVelocity = new TDNumber(this, getName(), "Measured Velocity");
        m_TDmeasuredCurrent = new TDNumber(this, getName(), "Measured Current");
    }

    public static Shooter getInstance() {
        if (m_Shooter == null) m_Shooter = new Shooter();
        return m_Shooter;
    }

    public void setTargetVelocity(double velocity) {
        m_TDvelocity.set(velocity);
    }

    @Override
    public void periodic() {
        if (Constants.ShooterConstants.kTunePID) {
            if (m_TDshootP.get() != m_shootP ||
                m_TDshootI.get() != m_shootI ||
                m_TDshootD.get() != m_shootD) {
                m_shootP = m_TDshootP.get();
                m_shootI = m_TDshootI.get();
                m_shootD = m_TDshootD.get();

                m_leftConfig.closedLoop.pid(m_shootP, m_shootI, m_shootD);

                m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }

            if(m_TDshootKv.get() != m_shootKv ||
               m_TDshootKs.get() != m_shootKs)
            {
                m_shootKv = m_TDshootKv.get();
                m_shootKs = m_TDshootKs.get();
                m_Feedforward = new SimpleMotorFeedforward(m_shootKs, m_shootKv);
            }
        }

        double requestedVelocity = m_TDvelocity.get();
        double ffVoltage = m_Feedforward.calculate(requestedVelocity);
        m_leftMotor
            .getClosedLoopController()
            .setSetpoint(
                requestedVelocity, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0, 
                ffVoltage);
        
        m_TDmeasuredVelocity.set(m_leftMotor.getEncoder().getVelocity());
        m_TDmeasuredCurrent.set(m_leftMotor.getOutputCurrent());
    }
}
