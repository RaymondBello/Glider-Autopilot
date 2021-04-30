import java.awt.Dimension;
import java.io.IOException;
import javax.swing.JEditorPane;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;


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
