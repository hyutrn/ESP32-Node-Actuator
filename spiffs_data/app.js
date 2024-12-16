const ip_test = '/';

var ssid_available = []
var ip_gateway_available = []
const ssid_element = document.getElementById('ssid');
const ip_gateway_element = document.getElementById('ip-gateway');

ssid_element.addEventListener('change', (event) => {
  const ssid_other_element = document.querySelector('input[name="ssid-other"]');
    if (event.target.value === 'other') {
      ssid_other_element.removeAttribute('hidden');
    }
    else {
        ssid_other_element.setAttribute('hidden', '');
    }
});

// Register ip gateway

const scan_button = document.getElementsByClassName('buton-scan');

scan_button[0].addEventListener('click', function (e) {
  e.preventDefault();
  ssid_element.innerHTML = '';
  var option = document.createElement('option');
  option.text = 'Scanning...';
  option.setAttribute('disabled', '');
  option.setAttribute('selected', '');
  ssid_element.appendChild(option);
  // Create a GET method on /ssid_available
  fetch(ip_test + 'ssid_available', {
    method: 'GET',
  })
  .then(response => response.json())
  .then(data => {
    console.log(data);
    // If the response is 200 (success), display the available SSID
    if (data.status === 200) {
      ssid_available = data.ssid_available;
      ssid_element.innerHTML = '';
      option = document.createElement('option');
      option.value = 'none';
      option.text = '-- Select SSID --';
      option.setAttribute('disabled', '');
      option.setAttribute('selected', '');
      ssid_element.appendChild(option);
      option = document.createElement('option');
      option.value = 'other';
      option.text = 'Type a SSID';
      ssid_element.appendChild(option);
      for (var i = 0; i < ssid_available.length; i++) {
        option = document.createElement('option');
        option.value = i;
        option.text = ssid_available[i].ssid;
        ssid_element.appendChild(option);
      }
      alert('Scan completed');
    }
    else if (data.status === 204) {
      // If the response is 204 (no content), notify the user that no SSID is available
      ssid_element.innerHTML = '';
      option = document.createElement('option');
      option.value = 'none';
      option.text = '-- Select SSID --';
      option.setAttribute('disabled', '');
      option.setAttribute('selected', '');
      ssid_element.appendChild(option);
      option = document.createElement('option');
      option.value = 'other';
      option.text = 'Type a SSID';
      ssid_element.appendChild(option);
      alert('No SSID available');
    }
  })
  .catch(error => {
    console.error('Error:', error);
    // Handle errors here, if necessary
  });
});

document.querySelectorAll('form')[0].addEventListener('submit', function (e) {
  e.preventDefault();
  const loginButton = document.querySelector('button[name="login-wifi"]');
  loginButton.setAttribute('disabled', '');
  loginButton.innerHTML = 'Logging in...';
  var ssidValue = document.querySelector('#ssid').value;
  var ssidOtherValue = document.querySelector('input[name="ssid-other"]').value;
  if (ssidValue === 'other') {
    ssidValue = ssidOtherValue;
  }
  else {
    ssidValue = ssid_available[ssidValue].ssid;
  }
  var passwordValue = document.querySelector('input[name="password"]').value;
  // Create a POST method on /login_wifi?ssid=ssidValue&password=passwordValue
  fetch(ip_test + 'login_wifi?ssid=' + ssidValue + '&password=' + passwordValue, {
    method: 'POST',
  })
  .then(response => response.json())
  .then(data => {
    console.log(data);
    // If the response is 401 (unauthorized), require the user to re-enter the password
    if (data.status === 401) {
      alert('Wrong SSID or password, please try again.');
      loginButton.removeAttribute('disabled');
      loginButton.innerHTML = 'Login';
    }
    // If the response is 200 (success), redirect to the home page
    else if (data.status === 200) {
      alert('Successful login, move on to register with gateway!');
      loginButton.innerHTML = 'Done!'
    }
  })
  .catch(error => {
    console.error('Error:', error);
    // Handle errors here, if necessary
  });
});

document.querySelectorAll('form')[1].addEventListener('submit', function (e) {
  e.preventDefault();
  const register_button = document.querySelector('button[name="register-gateway"]');
  register_button.setAttribute('disabled', '');
  register_button.innerHTML = 'Registering';
  var ip_gateway_value = document.querySelector('input[name="ip-gateway"]').value;
  console.log(ip_gateway_value);
  fetch(ip_test + 'register_node?ip_gateway=' + ip_gateway_value, {
    method: 'POST',
  })
  .then(response => response.json())
  .then(data => {
    console.log(data);
    // If the response is 401 (unauthorized), require the user to re-enter
    if (data.status === 401) {
      alert('Wrong IP Gateway, please try again.');
    }
    // If the response is 200 (success), redirect to the home page
    else if (data.status === 200) {
      register_button.innerHTML = 'Registered';
      alert('Registered with gateway successfully, restarted in minutes!');
    }
  })
  .catch(error => {
    console.error('Error:', error);
    // Handle errors here, if necessary
  });
});