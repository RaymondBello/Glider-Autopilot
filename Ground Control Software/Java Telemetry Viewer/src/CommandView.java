import java.awt.Dimension;
import javax.swing.JPanel;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.JTabbedPane;
import javax.swing.JScrollPane;
import java.awt.Font;
import net.miginfocom.swing.MigLayout;
import javax.swing.JTextArea;



@SuppressWarnings("serial")
public class CommandView extends JPanel {
   
   static CommandView instance = new CommandView();

   
   JTextField commandTextField;  // Box where all telemetry commands to be sent are entered 
   JButton sendCommandButton;    // Sends the telemetry command specified in 'commandTextField'
   JButton clearSerialMonitorButton; // Clears serial monitor box
   JTabbedPane serialMonitorPane;
   
   Dimension preferredSize;      // Dimension initialization 

   private CommandView() {
      super();

      // Set layout style
      setLayout(new MigLayout());

      // 
      add(new JLabel("Enter Telemetry Command "));
      commandTextField = new JTextField("", 10);
      add(commandTextField, "grow");
      sendCommandButton = new JButton("Send Command");
      add(sendCommandButton);
      clearSerialMonitorButton = new JButton("Clear monitor");
      add(clearSerialMonitorButton, "wrap");

      // add(Box.createHorizontalStrut(Theme.padding));
      // tabs for displaying serial Monitor
      serialMonitorPane = new JTabbedPane();
      add(serialMonitorPane, "grow, span");


      preferredSize = getPreferredSize();

      
      updateSerialMonitor();

      setVisible(true);

   }
   
   public void updateSerialMonitor() {

      serialMonitorPane.removeAll();

      JTextArea serialMonitor = new JTextArea();
      serialMonitor.setEditable(false);
      serialMonitor.setTabSize(4);
      serialMonitor.setFont(new Font("Fira Code", Font.PLAIN, 17));

      serialMonitorPane.add("Serial Monitor Console", new JScrollPane(serialMonitor));
      serialMonitor.setText("Empty UART Console");
      serialMonitor.append("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

      // scroll back to the top
      serialMonitor.setCaretPosition(0);

   }
   
}
