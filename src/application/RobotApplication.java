package application;


import hehua.GsonUtil;
import hehua.ProtocolBean;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.util.LinkedList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import javax.inject.Inject;

import parameters.MovePCommandParamerer;

import com.kuka.med.mastering.Mastering;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import commands.MoveP;

import core.CommandHandlerRepeater;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RobotApplication extends RoboticsAPIApplication {

	@Inject
	private LBR lbr;
	private Tool tool;
	@SuppressWarnings("unused")
	@Inject
	private Mastering mastering;
	
	public static boolean isDebug = true;
	
	protected CommandHandlerRepeater commandHandlerRepeater = new CommandHandlerRepeater();

	// Debug
	
	private boolean m_stop = false;
	public boolean isSendMaster = false;
	private Socket m_socket = null;
	private BufferedReader m_reader = null;
	private OutputStreamWriter m_writer = null;
	//private ProtocolProcess m_processer = null;

	private LinkedList<String>  msgQueque = new LinkedList<String>();
	private ReadWriteLock msgLock = new ReentrantReadWriteLock();
	// EndDebug
	
	
	
	@Override
	public void initialize() {

		// <1		
		this.lbr = getContext().getDeviceFromType(LBR.class);
		
		this.mastering = new Mastering(this.lbr);
		
		// <2
		ITransformation tans = Transformation.ofDeg(0, 0, 20, 0, 0, 0);
		LoadData load_lbr =  new LoadData();
		load_lbr.setCenterOfMass(tans);
		load_lbr.setMass(0.676);
		
		// <3
		this.tool = new Tool("tool", load_lbr);
		this.tool.attachTo(lbr.getFlange());
		this.tool.addDefaultMotionFrame("cubeOrigin", Transformation.ofTranslation(0, 0, 66));
		this.tool.addChildFrame("Finger", Transformation.ofTranslation(0, 48, 157));
		this.tool.addChildFrame("RobotUserTool", Transformation.ofTranslation(0, 200, 170));
		
		// 
		if(true == this.InitializeCoreModules() && RobotApplication.isDebug) {
			System.out.println("[INFO] " + "Finished initializing the core modules.");
		}else if(RobotApplication.isDebug) {
			System.out.println("[INFO] " + "Finished initializing the core modules.");
		}
		
		
	}

	@Override
	public void run() {
		this.m_socket = new Socket();
		this.ReConnect();
		
		Thread thrd = new Thread(new Runnable() {
			@Override
			public void run() {
				long oldTime = System.currentTimeMillis();
				while (!m_stop) {
					String line = new String();
					try {
						long timeOut = System.currentTimeMillis() - oldTime;
						if (0 != oldTime && timeOut > 5 * 1000) {			
							
							ReConnect();
							Thread.sleep(3000);
							if (m_socket.isConnected()) {
								oldTime = 0;				
							}
						}
						if (m_reader != null && m_reader.ready()) {
							line = m_reader.readLine();
							//logger.info(line);
						}
						if (0 == line.compareTo("heartBeat")) {
							oldTime = System.currentTimeMillis();
							//logger.info("oldTime"+String.valueOf(oldTime));
							continue;
						}
						if (!line.isEmpty()) {
							msgLock.writeLock().lock();
							msgQueque.offer(line);
							msgLock.writeLock().unlock();
						}
					} catch (Exception ex) {
						ex.printStackTrace();
					}

				}
			}
		});
		thrd.start();
		
		//
		try {
			while (!m_stop) {										
				ProtocolBean msgBean = getMsgBean();
				if (msgBean != null) {
					this.commandHandlerRepeater.PushCommandToHandler(msgBean.getOperateType());
				}
			}

			m_reader.close();
			m_writer.close();		
			m_socket.close();
		} catch (IOException e) {

		}
	}
	
	protected boolean InitializeCoreModules() {
		boolean isInitialize = true;
		
		isInitialize &= this.InitializeCoreRuntimeEnvironmentModules();
		// Initialize the runtime environment.
		if(true == isInitialize && RobotApplication.isDebug) {
			System.out.println("[INFO] " + "Finished initializing the runtime environment.");
		}else if(RobotApplication.isDebug) {
			System.out.println("[ERROR] " + "Failed to initialize the runtime environment.");
		}
		
		// Initialize Command Factory.
		isInitialize &= this.InitializeCoreCommandFactoryModules();
		if(true == isInitialize && RobotApplication.isDebug) {
			System.out.println("[INFO] " + "Finished initializing the command factory.");
		}else if(RobotApplication.isDebug) {
			System.out.println("[ERROR] " + "Failed to initialize the command factory.");
		}
		
		// Initialize Command Parameter Factory.
		isInitialize &= this.InitializeCoreCommandFactoryParameterModules();
		if(true == isInitialize && RobotApplication.isDebug) {
			System.out.println("[INFO] " + "Finished initializing the command parameter factory.");
		}else if(RobotApplication.isDebug) {
			System.out.println("[ERROR] " + "Failed to initialize the command parameter factory.");
		}
		return isInitialize;
	}
	
	/**
	 * Initialize the runtime environment of the kernel module.
	 * 
	 * <p>
	 * The runtime environment refers to the resources that the kernel module 
	 * depends on for automation. These resources may be robot arm example 
	 * objects or application instance objects.
	 * </p>
	 * 
	 * <table border="1" align="center" cellspacing="0" cellpadding="16" width="500">
	 *   <caption>Runtime Registration Form</caption>
	 *   <tr>
	 *     <th>KeyName    </th>
	 *     <th>Describe   </th>
	 *   </tr>
	 *   
	 *   <tr>
	 *     <td>RobotApplication</td>
	 *     <td>Robot application instance object.</td>
	 *   </tr>
	 * </table>
	 * 
	 * @see units.CommandParameterFactory
	 */
	protected boolean InitializeCoreRuntimeEnvironmentModules() {
		boolean isInitialize = true;
		isInitialize &= this.commandHandlerRepeater.commandParameterFactory.RegisterRunTimeProperty(this);
		isInitialize &= this.commandHandlerRepeater.commandParameterFactory.RegisterRunTimeProperty(this.lbr);
		return isInitialize;
	}

	/**
	 * Initialize the command factory of the kernel module.
	 * 
	 * <p>
	 * The command factory is the manufacturer of the command entity object, and 
	 * the correct manufacturing target of the command factory The command entity 
	 * object requires a work instruction, which is accessed through the command 
	 * factory registration interface.
	 * </p>
	 * 
	 * <table border="1" align="center" cellspacing="0" cellpadding="16" width="500">
	 *   <caption>Runtime Registration Form</caption>
	 *   <tr>
	 *     <th>CommandName</th>
	 *     <th>Describe   </th>
	 *   </tr>
	 *   
	 *   <tr>
	 *     <td>MoveP</td>
	 *     <td>Make the mechanical arm move a line, which is expressed by the starting 
	 *     point (generally the current point position of the mechanical arm) and the 
	 *     target point (the stop position of the mechanical arm).</td>
	 *   </tr>
	 * </table>
	 * 
	 * @see units.CommandFactory
	 */
	protected boolean InitializeCoreCommandFactoryModules() {
		boolean isInitialize = true;
		isInitialize &= this.commandHandlerRepeater.commandFactory.RegisterProduct(new MoveP());
		return isInitialize;
	}

	/**
	 * Initialize the command accessory parameter product of the kernel module.
	 * 
	 * <p>
	 * The parameter product is an accessory of the command product, and a command 
	 * product should correspond to a parameter accessory to assist the normal use 
	 * of the target function of the command product.
	 * </p>
	 * 
	 * <table border="1" align="center" cellspacing="0" cellpadding="16" width="500">
	 *   <caption>Runtime Registration Form</caption>
	 *   <tr>
	 *     <th>CommandName</th>
	 *     <th>CommandParameterName</th>
	 *     <th>Describe   </th>
	 *   </tr>
	 *   
	 *   <tr>
	 *     <td>MoveP</td>
	 *     <td>MovePCommandParamerer</td>
	 *     <td>A line parameter represented by the starting point (usually the current 
	 *     point position of the manipulator) and the target point (the stop position 
	 *     of the manipulator).</td>
	 *   </tr>
	 * </table>
	 * 
	 * @see units.CommandParameterFactory
	 */
	protected boolean InitializeCoreCommandFactoryParameterModules() {
		boolean isInitialize = true;
		isInitialize &= this.commandHandlerRepeater.commandParameterFactory.RegisterParameter("MoveP", new MovePCommandParamerer());
		return isInitialize;
	}
	
	// TODO: Debug
	public void  ReConnect() {
		try {
			SocketAddress address = new InetSocketAddress("172.31.1.148", 30009);

			m_socket.close();
			Thread.sleep(100);
			m_socket = new Socket();
			m_socket.connect(address, 0);
			
			m_reader = new BufferedReader(new InputStreamReader(
					m_socket.getInputStream()));
			m_writer = new OutputStreamWriter(new DataOutputStream(
					m_socket.getOutputStream()));

		} catch (Exception ex) {
			
		}
	}
	
	// TODO: Debug
	public  ProtocolBean getMsgBean() {
		msgLock.readLock().lock();
		String line = msgQueque.poll();
		msgLock.readLock().unlock();
		
		ProtocolBean msgBean = null;
		if (line != null && !line.isEmpty()) {			
			msgBean = GsonUtil.json2Bean(line, ProtocolBean.class);
		}
		return msgBean;
	}
}