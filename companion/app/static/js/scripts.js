$(document).ready(function() {
    // Handle Configuration Form Submission
    $('#config-form').submit(function(event) {
        event.preventDefault();
        $.ajax({
            url: '/config',
            type: 'POST',
            data: $(this).serialize(),
            success: function(response) {
                alert(response.message);
            }
        });
    });
 
    // Handle Parameters Form Submission
    $('#parameters-form').submit(function(event) {
        event.preventDefault();
        // Implement the AJAX call to update parameters
        $.ajax({
            url: '/update-parameter',
            type: 'POST',
            data: $(this).serialize(),
            success: function(response) {
                alert('Parameter updated successfully!');
            }
        });
    });
});
 
const target_button = document.getElementById('set_tracking_target');

target_button.addEventListener('click', async _ => {
  try {     
    const response = await fetch('set_tracking_target', {
      method: 'post',
      body: {
        // Your body
      }
    });
    console.log('Completed!', response);
  } catch(err) {
    console.error(`Error: ${err}`);
  }
});