import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.io.IOException;
import java.util.Hashtable;

import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JEditorPane;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JTextField;
import net.miginfocom.swing.MigLayout;


@SuppressWarnings("serial")
public class MapView extends JEditorPane {

   static MapView instance = new MapView();
   Dimension preferredSize;
   JScrollPane scrollPane;


   private MapView() {
      super();
      setEditable(true);
      // setLayout(new MigLayout());

      try {
         setPage("http://www.google.com");
      } catch (IOException e) {
         //TODO: handle exception
         setContentType("text/html");
         setText("<html>Could not load</html>");
      }
      
      setPreferredSize(new Dimension(400, 400));
      // scrollPane = new JScrollPane(this);
      
      setVisible(true);
   }
   
}
