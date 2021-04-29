import java.awt.Dimension;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import net.miginfocom.swing.MigLayout;



@SuppressWarnings("serial")
public class CommandView extends JPanel {
   
   static CommandView instance = new CommandView();

   JTextField commandTextField;
   JButton sendCommandButton;
   Dimension preferredSize;

   private CommandView() {
      super();
      setLayout(new MigLayout());

      add(new JLabel("Enter Telemetry Command "));
      commandTextField = new JTextField("",10);
      add(commandTextField, "grow");
      sendCommandButton = new JButton("Send Command");
      add(sendCommandButton, "wrap");

      preferredSize = getPreferredSize();

      setVisible(true);

   }


   
}
